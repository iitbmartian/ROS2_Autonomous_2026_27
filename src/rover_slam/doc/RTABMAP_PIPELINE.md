# Putting RTAB-Map on the Gazebo rover

How `rover_slam` was built and wired into `rover_gazebo`, what had to change in the
simulation to make it work, why each parameter is set the way it is, and what to do when the
map comes out wrong.

---

## 1. Where this came from

`rover_gazebo` lives on the branch `pradyun/rover_gazebo-urdf-sim-bringup`, not on `main`.
This work is branched from that commit, so `rover_slam` and the simulation it depends on are
in the same tree:

```bash
git clone https://github.com/iitbmartian/ROS2_Autonomous_2026_27.git
cd ROS2_Autonomous_2026_27
git checkout pradyun/rover_gazebo-urdf-sim-bringup
git checkout -b sohan/rover_slam-rtabmap-bringup
```

The consequence is that this branch cannot merge until Pradyun's does. It is the honest
option: the alternative, copying `rover_gazebo` onto a branch off `main`, duplicates his work
and guarantees a conflict when his PR lands.

Built and tested on ROS 2 Jazzy, Gazebo Harmonic 8.11, RTAB-Map 0.22.1 from apt. Everything
needed is already packaged; nothing is built from source.

---

## 2. The bug this had to fix first

RTAB-Map would not have produced a usable map against the simulation as it stood, and the
reason had nothing to do with RTAB-Map.

`rover_gazebo/urdf/rover_sensors.xacro` declared the RGB-D camera like this:

```xml
<sensor name="rgbd_camera" type="rgbd_camera">
  <gz_frame_id>camera_link</gz_frame_id>
```

`gz_frame_id` is the frame Gazebo stamps into the header of every image and `CameraInfo` the
sensor publishes. `camera_link` is a ROS-convention frame: x forward, y left, z up. You can
confirm that from the URDF alone, since the rotation from `base_link` to `camera_link` works
out to the identity.

Every ROS image consumer, RTAB-Map included, reads the frame named in an image header as an
**optical** frame: z forward, x right, y down. That is REP 103, and it is not negotiable,
because reprojecting a depth image into 3D depends on it. Handed `camera_link`, RTAB-Map
reprojects each depth pixel through a frame rotated ninety degrees from the one the data is
actually in, and the reconstructed cloud lands on its side.

The file already defined the correct frame, one joint further down:

```xml
<link name="camera_optical_frame"/>
<joint name="camera_optical_joint" type="fixed">
  <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
  <parent link="camera_link"/>
  <child link="camera_optical_frame"/>
</joint>
```

That `rpy` is exactly the ROS-to-optical rotation. Nothing was publishing into it. The fix is
one line, pointing the sensor at the frame that was already there:

```xml
<gz_frame_id>camera_optical_frame</gz_frame_id>
```

The lidar keeps `lidar3d_link`, correctly: point clouds carry their own geometry and are not
optical-frame data.

This was the first change to Pradyun's package. There is now a second: `marsyard.sdf`,
its generator `tools/make_marsyard.py` and the texture under `models/MarsyardGround`,
added so the visual path has a world it can run on. Both need calling out in the PR. It is worth
noticing that the file's own header comment already said the camera published "plus the
REP-103 optical frame the image data is published in". The comment was right; the code was
wrong.

**Robot State Publisher, not Gazebo, is what makes the frame resolvable.** `camera_optical_frame`
is a massless link, so sdformat folds it into its parent when the URDF is converted. That does
not matter, because `gz_frame_id` is only a string written into a message header, and the
`camera_link` → `camera_optical_frame` transform reaches TF from `robot_state_publisher`,
which publishes every fixed joint in the URDF.

---

## 3. What the pipeline looks like

```
Gazebo
  │  rgbd_camera ──┐
  │  gpu_lidar ────┤  ros_gz_bridge
  │  imu ──────────┘
  ▼
/camera/image_raw  /camera/depth_image  /camera/camera_info   /lidar/points   /imu/data
        │                  │                    │                   │            │
        └──────────────────┴────────────────────┘                   │            │
                           ▼                                        │            │
                    rgbd_sync ──► /rgbd_image ───────────┬───────────┘            │
                                                          │                       │
                                        rgbd_odometry ◄───┤◄──────────────────────┘
                                              │           │
                                    /rtabmap/odom         │
                                    TF odom→base_footprint│
                                              │           │
                                              └──────► rtabmap
                                                          │
                                          /map  /cloud_map  TF map→odom
```

`rgbd_sync` exists so RGB, depth and intrinsics are matched once into a single `RGBDImage`,
rather than having the odometry node and the mapper each synchronise three topics separately.

### Which topic carries what

No topic here has an `/rtabmap` prefix. RTAB-Map's documentation writes `/rtabmap/cloud_map`
because its upstream launch files put the node in an `rtabmap` namespace. This one sets none,
so `/map` lands where Nav2 looks for it. `/rtabmap/cloud_map` does not exist.

The outputs fall into two families, and confusing them is the single easiest mistake to make
here.

| Topic | Type | What it actually is |
|---|---|---|
| `/lidar/points`, `/camera/*`, `/imu/data` | raw | straight off the Gazebo bridge |
| `/odom` | raw | Gazebo ground truth, always published, never an estimate |
| `/rgbd_image` | raw | the three camera topics synced into one message |
| `/map` | grid | the 2D OccupancyGrid Nav2 plans on |
| `/cloud_map` | grid | **occupancy cells as points, not raw scans** |
| `/cloud_ground`, `/cloud_obstacles` | grid | the same cells split by classification |
| `/mapData` | graph | poses **plus each node's compressed raw scan** |
| `/mapGraph`, `/mapPath`, `/info` | graph | pose graph, trajectory, loop-closure counters |
| `/rtabmap/odom` | estimate | only exists under `odom_source:=visual` or `:=lidar` |

The grid family is built by RTAB-Map's `MapsManager` out of occupancy cells. Since `Grid/3D`
is false, those cells are all projected to one height, so `/cloud_map` is flat by
construction: measured at 325 points spanning 8 cm on a real run. That is why RViz shows a
red outline where `rtabmap_viz` shows a scene.

The viewer is not reading any of those. Its 3D Map pane subscribes to `/mapData`, decompresses
each node's raw scan, and assembles the result inside its own process. Nothing republishes
that assembly, which is the whole reason the two look different.

To get the same thing on a topic, launch with `dense_map:=true`. That starts `rtabmap_util`'s
`map_assembler` in a `dense_map` namespace, where it re-derives the local grids in 3D from the
raw scans in `/mapData` and publishes `/dense_map/cloud_map`. It runs in its own process
deliberately, so the cost stays off the mapping loop. See the note below on why `Grid/3D` is
not simply set to true on the mapper.

### Frames

`rover_gazebo` publishes everything from `base_footprint` down and, deliberately, nothing
above it. Its integration guide says so outright: "Nothing publishes `odom` to
`base_footprint`. That transform is your estimator's job." This package is that estimator.

```
map                  ← rtabmap
  odom               ← rgbd_odometry, or gt_odom_tf.py
    base_footprint   ← rover_gazebo, from here down
      base_link
        chassis_link
```

### One trap worth knowing about

`rgbd_odometry` publishes on `odom` by default. So does the `ros_gz_bridge`, carrying
Gazebo's ground truth. Left alone, two publishers interleave on one topic and every
subscriber gets alternating estimated and true poses with nothing to tell them apart. It
would not error; it would just quietly poison `rover_ekf` and anything else reading `/odom`.

So the odometry output is remapped to `/rtabmap/odom`, and `/odom` stays ground truth, as
`rover_gazebo/doc/INTEGRATION.md` describes it.

---

## 4. Files

| File | What it does |
|---|---|
| `rover_bringup/launch/rtabmap.launch.py` | the SLAM stack; attaches to an already-running simulation |
| `rover_bringup/launch/pipeline.launch.py` | simulation, EKF, SLAM and Nav2 in one command |
| `rover_slam/gt_odom_tf.py` | rebroadcasts Gazebo's `/odom` as TF, for `odom_source:=ground_truth` |
| `rviz/rover_slam.rviz` | map, cloud map, odometry, TF, camera |
| `package.xml`, `CMakeLists.txt` | `ament_cmake`, install-only, matching `rover_gazebo` |

### Why the parameters are in the launch file and not a YAML

RTAB-Map's own parameters, the ones with a slash in the name, must be passed as **strings**:
`'Grid/RayTracing': 'true'`, never a Python bool. The node forwards them into RTAB-Map's
string-keyed parameter map, and a real bool or float is mishandled there. Upstream never puts
them in YAML for exactly this reason; every launch file under `rtabmap_demos` and
`rtabmap_examples` uses Python dicts, so this one does too.

The structure follows
`/opt/ros/jazzy/share/rtabmap_demos/launch/husky/husky_slam3d.launch.py`, which is the same
problem: a simulated robot carrying a 3D lidar and a camera.

### Why an OpaqueFunction

The node list is built inside an `OpaqueFunction` rather than with `IfCondition`. Two reasons.
The database-reset flag has to be either present or absent in `rtabmap`'s argv, and a
substitution that evaluates to `''` leaves an empty argument sitting there. And the visual and
ground-truth paths differ by more than one node, so plain Python `if` reads better than four
conditions.

The cost: `OpaqueFunction` implements only `execute`, so `ros2 launch --print-description`
cannot expand it and prints no nodes at all. That is not an error. Validate by running the
launch, not by printing it.

---

## 5. Parameter choices

### Both sensors, and how they are used

`Reg/Strategy=2` is visual registration refined by ICP: image features propose the match, the
lidar geometry sharpens it. This is the reason both sensors are wired in rather than one.

`Reg/Strategy` is set on the mapping node **only**. `rgbd_odometry` supports visual
registration alone and logs "RGBD odometry works only with Reg/Strategy=0. Ignoring value 2."
if it sees it.

### Lidar odometry, and why it was added

The first end-to-end run showed `rgbd_odometry` never initialising. It needs 15 tracked
features and found about 8, because the worlds are untextured. That was not a tuning
problem and no threshold would have fixed it honestly; lowering `Vis/MinInliers` far enough
to initialise would only have produced an estimate built on noise.

`odom_source:=lidar` runs `rtabmap_odom/icp_odometry` instead. ICP registers geometry rather
than appearance, so texture is irrelevant to it, and the rover already carries the 3D lidar.
Parameters follow `rtabmap_examples/launch/lidar3d.launch.py`:

| Parameter | Value | Why |
|---|---|---|
| `Odom/ScanKeyFrameThr` | `0.4` | new keyframe once the scan overlap drops below 40% |
| `OdomF2M/ScanSubtractRadius` | `0.1` | matches `Icp/VoxelSize`, so the local map does not accumulate duplicate points |
| `OdomF2M/ScanMaxSize` | `15000` | cap on the local map, roughly three scans' worth |
| `Icp/CorrespondenceRatio` | `0.01` | 360x16 is a sparse cloud, so demand little overlap per match |
| `scan_cloud_max_points` | `5760` | the 360x16 declared in `rover_sensors.xacro`. Stated because `icp_odometry` infers it and warns when unset |

ICP has its own blind spot, the mirror of the visual one: a perfectly flat plane constrains
nothing in x, y or yaw, so on `flat.sdf` it drifts freely. Section 6 has the pairing.

### The occupancy grid, which is the 3D → 2D squish

| Parameter | Value | Why |
|---|---|---|
| `RGBD/CreateOccupancyGrid` | `true` | **Defaults to false.** Without it no grid is built and `/map` never appears, with nothing in the log to say why |
| `Grid/Sensor` | `2` | 0 is laser scan, 1 is depth image, 2 is both. The lidar gives 360° range, the camera gives close ground detail the roof lidar cannot see over the chassis |
| `Grid/3D` | `false` | this is the squish: project the cloud onto xy and emit a plain `OccupancyGrid`; see below before changing it |
| `Grid/RayTracing` | `true` | fill known-free space between the rover and each hit, instead of leaving it unknown |
| `Grid/NormalsSegmentation` | `false` | height thresholds instead of surface normals, see below |
| `Grid/MaxGroundHeight` | `0.15` | clears the 10 cm ledge in `ledge.sdf` |
| `Grid/MaxObstacleHeight` | `1.0` | ignore anything the rover can drive under |
| `Grid/Footprint*` | 1.0 × 0.8 × 0.5 | do not map the rover's own wheels as obstacles |
| `Grid/RangeMax` | `8.0` | well inside the sensor's 30 m, because of how few rings reach the ground; see below |

The normals-versus-height choice matters for this rover specifically. `ledge.sdf` is a 10 cm
step the suspension is designed to climb, and a normals-based segmenter reads its vertical
face as a wall. Height thresholding lets the rover plan over what it can actually drive over.

#### Why Grid/3D stays false, and what to use instead

Setting `Grid/3D` to true is the obvious way to make `/cloud_map` three-dimensional, and it
was measured on an identical drive rather than assumed. It is too expensive on the mapping
loop:

| | `/cloud_map` points | z span | RTAB-Map iteration | Resident memory |
|---|---|---|---|---|
| `Grid/3D` false | 325 | 0.08 m | 0.038 s | — |
| `Grid/3D` true | 32726 | 1.05 m | **1.50 s** | 1.3 GB |
| `dense_map:=true` | 7521 on `/dense_map/cloud_map` | **1.81 m** | 0.18 s | 410 MB + 307 MB |

`Rtabmap/DetectionRate` is 1 Hz, so a 1.50 s iteration means the mapper stops keeping up:
measured processing delay rose from 0.14 s to 0.85 s. A slower SLAM loop is a bad trade for a
nicer picture.

`dense_map:=true` gets the 3D cloud without that. `map_assembler` runs as a separate process,
so its assembly cannot stall the mapper, and it is free to use a coarser 0.1 m cell since
nothing plans on its output. It also reaches the logo plate's full 1.81 m, where the
`Grid/3D` true run clipped at `Grid/MaxObstacleHeight`.

#### Why the grid stops at 8 m

The lidar is 16 rings over a ±15° fan at roughly 0.75 m above ground, so only the downward
half ever lands, 2° apart. Within range the ground is sampled by seven concentric rings and
nothing in between:

| Ring | −15° | −13° | −11° | −9° | −7° | −5° | −3° |
|---|---|---|---|---|---|---|---|
| Ground radius | 2.80 m | 3.25 m | 3.86 m | 4.74 m | 6.11 m | 8.57 m | 14.31 m |

The next ring up would touch down at 43 m, past the sensor's own 30 m limit. Because
classification is height-only against `Grid/MaxGroundHeight`, a whole ring flips from ground
to obstacle as soon as the chassis pitches by `atan(0.15 / radius)`: 0.6° at 14.31 m, 1.0° at
8.57 m, 1.4° at 6.11 m. The rover pitches well past that on Gazebo Harmonic, which painted a
false red obstacle ring around the edge of everything explored. Capping at 8 m drops the two
worst rings and keeps five.

That ring structure is also why the 3D Map view in `rtabmap_viz` looks like scattered points
rather than a surface. That pane assembles `/mapData`, not `/cloud_map`, and draws each
occupancy cell as a single point on the 5 cm `Grid/CellSize` lattice. Along the outermost
kept ring the samples are 15 cm apart, three cells. Nothing in this path is meshed, so it
cannot render as a surface.

### Attitude, not flat-floor

`Reg/Force3DoF` stays **false**, which is where this departs from the husky demo it is
otherwise modelled on. That demo forces 3 DoF because a warehouse floor is flat. This rover
exists to be driven over `ledge.sdf` and `bars.sdf`, where roll and pitch are signal.

`Optimizer/GravitySigma=0.3`, against the default GTSAM optimiser, pins roll and pitch using
the simulated IMU, so the map does not slowly tilt over a long run. `wait_imu_to_init` on the
odometry node holds initialisation until the first IMU message, so the very first pose is
already gravity-aligned rather than corrected later.

### Talking to the bridge

`qos: 1`, reliable. `ros_gz_bridge` publishes reliable; RTAB-Map defaults to best-effort,
which matches nothing the bridge sends and drops every message without an error.

`approx_sync: true` with `approx_sync_max_interval: 0.1`. The camera runs at 15 Hz and the
lidar at 5 Hz, so exact timestamp sync would never fire.

`wait_for_transform: 0.2`, up from the 0.1 s default, because the clock arrives across the
bridge and TF can land late.

### One thing that cannot be configured

`subscribe_odom_info` would give the mapper the odometry quality information it uses to weigh
each link, and it is deliberately **not** set. RTAB-Map synchronises through a fixed set of
`message_filters` combinations declared in
`/opt/ros/jazzy/include/rtabmap_sync/CommonDataSubscriber.h`. `rgbdOdomScan3d` exists and
`rgbdOdomInfo` exists, but `rgbdOdomScan3dInfo` does not. Subscribing to RGB-D and lidar
together rules `OdomInfo` out. Asking for it anyway just logs "subscribe_odom_info
ignored...". Do not add it back without dropping one of the two sensors.

---

## 6. Running it

```bash
colcon build --symlink-install --packages-select rover_gazebo rover_slam rover_bringup
source install/setup.bash
ros2 launch rover_bringup pipeline.launch.py odom_source:=ground_truth rviz:=true
ros2 run rover_gazebo teleop_rover.py     # another terminal, keep it focused
```

`W`/`S` drive, `A`/`D` turn, `1`/`2`/`3`/`4` pick Ackermann, crab, spot turn or
explicit, space stops. Explicit is the swerve mode: `A`/`D` aim all four wheels by
hand, `W`/`S` then drive along that aim, and `C` sweeps them back to straight.
These are not the keys `teleop_twist_keyboard` sends, which is the usual reason the rover
sits still. The map only grows while the rover moves; watch `WM=` climb in the SLAM log.

Started by hand in a second terminal, as above, teleop falls back to the defaults in the
script. `rover_gazebo`'s `config/rover_control.yaml` only reaches it when the simulator
itself is launched with `teleop:=true`.

In RViz set Fixed Frame to `map`. The grey and black grid is `/map`, the coloured points
are `/cloud_map`.

### Choosing the odometry source

This matters more than anything else in the file, because every world in `rover_gazebo` is
a bare ground plane plus at most two boxes.

| World | Works with | Why |
|---|---|---|
| `empty_world.sdf` | `ground_truth`; `lidar` untested | the default world, a 9 m walled box |
| `flat.sdf` | `ground_truth` only | no texture for vision, no geometry for ICP |
| `bars.sdf` | `ground_truth`, `lidar` | two bars give ICP something to bite on |
| `ledge.sdf` | `ground_truth`, `lidar` | ramp and step, likewise |
| `husarion_world.sdf` | `ground_truth`, `lidar` | a 1.8 m tall logo plate; spawn clear of it with `x:=-13`, then drive along it, not into it |
| `marsyard.sdf` | `ground_truth`, `lidar`, **`visual`** | textured ground and 110 low rocks; the only world the visual path works on |

`ground_truth` is the default. It takes the simulator's own pose, so a mapping fault can be
told apart from an estimator fault, and it is the only source that works on `flat.sdf`.

`lidar` runs `rtabmap_odom/icp_odometry` on `/lidar/points`. It is the honest test, since
it uses a sensor the real rover carries:

```bash
ros2 launch rover_bringup pipeline.launch.py world:=bars.sdf odom_source:=lidar rviz:=true
```

`visual` runs `rgbd_odometry`. It needs 15 tracked features and finds about 8 against
blank surfaces, so on the four bare worlds it logs `15 visual features required to
initialize the odometry` forever and the mapper rejects every frame with `no odometry is
provided. Image 0 is ignored!`. Nothing was ever wrong with the settings; the worlds had
no texture.

`marsyard.sdf` is the world that fixes it, generated by `rover_gazebo`'s
`tools/make_marsyard.py`. Measured there: the warning never appears, `/rtabmap/odom`
publishes at 14.2 Hz, and around a 48 m two-lap circuit the estimate finished 0.06 m from
ground truth.

Localise against a map you already built:

```bash
ros2 launch rover_bringup pipeline.launch.py odom_source:=ground_truth localization:=true
```

Shut down with Ctrl-C and let it finish. A leftover `rtabmap` process fights the next run,
and two of them on one `/clock` makes sim time jump backwards, which shows up as
`Detected not valid consecutive stamps`. Clear it with
`pkill -f rtabmap; pkill -f "gz sim"`.

---

## 7. What was verified, and what was not

Verified without the simulator:

- `colcon build` clean for both packages.
- The processed xacro carries `<gz_frame_id>camera_optical_frame</gz_frame_id>`, so the frame
  fix reaches the model.
- Every launch branch starts clean with no simulator running: each node reaches a steady
  state waiting for topics. The mapper confirms it loaded `Grid/Sensor=2`, `Grid/3D=false`,
  `RGBD/CreateOccupancyGrid=true`, `Reg/Strategy=2`, and in localisation mode
  `Mem/IncrementalMemory=false`.

Verified in the simulator, headless, on `bars.sdf` with `odom_source:=lidar`:

| Check | Result |
|---|---|
| `/rtabmap/odom` rate | 4.86 Hz against a 5 Hz lidar |
| TF `map` → `base_footprint` | resolves |
| Map nodes after driving | grew from 1 to 31 |
| `/map` occupancy grid | 786 x 1207 cells at 0.05 m |
| Errors in the log | none |

`odom_source:=ground_truth` was verified the same way and also maps cleanly.

**Since verified**, all three on `marsyard.sdf` unless noted:

1. **Loop closure.** Two laps of a circle, then `rtabmap-info` on the saved database
   reporting `GlobalClosure: 1` alongside `LocalSpaceClosure: 2`. Live, `/info` carried
   `loop_closure_id` 72 matching node 28. The field is `loop_closure_id`, snake_case,
   not the `loopClosureId` of RTAB-Map's C++ API.

   The two link types are not the same thing and it is worth keeping them apart.
   `RGBD/ProximityBySpace` is on, so RTAB-Map also links nodes it is simply near, using
   geometry alone. The same drive on `husarion_world.sdf` produced 3 proximity links and
   **zero** global closures. Only a global closure is a visual loop closure.

2. **Localization mode.** Mapped `husarion_world.sdf` to a 66-node database, then
   relaunched with `localization:=true`: working memory loaded all 66 at startup and
   stayed there while the rover drove 23 m, and the database was left untouched.

   One trap found doing it. The database is written on clean shutdown only. A session
   killed with `SIGKILL` leaves the file at its empty startup size, and `rtabmap-info`
   then reports `Sessions: 0` with zero nodes. Let Ctrl-C finish, and check with
   `rtabmap-info` before blaming localization for a map that was never saved.

3. **`odom_source:=visual`.** Initialises immediately, publishes at 14.2 Hz, and
   finished a 48 m circuit 0.06 m from ground truth.

---

## 8. When it misbehaves

**`/map` never appears.** Check `RGBD/CreateOccupancyGrid` is `true`. It is off by default and
fails silently.

**Nothing arrives at all, no errors.** QoS mismatch. `qos` must be 1 against the bridge.

**The cloud is on its side, or depth lands in the wrong place.** The camera frame fix has been
lost. `ros2 topic echo /camera/camera_info --once` must say `camera_optical_frame`.

**"Did not receive data" warnings.** Timestamps are not lining up. Check `use_sim_time` is
true everywhere, that `/clock` is being published, and widen `approx_sync_max_interval`.

**Odometry keeps resetting.** The rover is probably in crab, spot or explicit mode, which
look nothing like the visual motion model. Explicit mode reaches the same wheel geometry
crab does whenever the aim sits near 90 degrees, so it fails the same way. `Odom/ResetCountdown` is 10, so it recovers rather than dying.
Confirm with `odom_source:=ground_truth`: if the map is fine there, the fault is in the
odometry, not the mapping.

**The map tilts over a long run.** `Optimizer/GravitySigma` and the IMU wiring. Check
`/imu/data` is arriving and `wait_imu_to_init` is set.

**`15 visual features required to initialize the odometry (only 8 extracted)`, forever.**
The scene has no texture. This is the normal state of every world in `rover_gazebo`. Switch
to `odom_source:=lidar` on `bars.sdf` or `ledge.sdf`, or `ground_truth` anywhere.

**`no odometry is provided. Image 0 is ignored!`** Always a consequence of the line above,
never a fault in the mapper. Nothing published `odom` → `base_footprint`, so fix the
odometry source first. `local map=0, WM=0` in the rate line is the same symptom.

**The rover does not move when you press keys.** `teleop_rover.py` uses `W`/`A`/`S`/`D`, not
the `i`/`j`/`k`/`l` that `teleop_twist_keyboard` sends. The teleop terminal also has to be
the focused one.

**`Detected not valid consecutive stamps`, sim time going backwards.** Two simulators or two
`rtabmap` processes are alive at once on one `/clock`. `pkill -f rtabmap; pkill -f "gz sim"`
and start again.

---

## 9. Pushing this

Two branches, and they are independent of each other.

**`sohan/workspace-colcon-build-fix`**, off `main`. A root `colcon build` failed for
everyone before it; the cause was an unanchored `lib/` in the root `.gitignore` silently
dropping a vendored binary. It touches the root `.gitignore`, the root `README.md` and four
`COLCON_IGNORE` markers inside `rover_drivers`, so Ram should review it. It can merge
immediately and does not wait on anything.

**`sohan/rover_slam-rtabmap-bringup`**, this work.

```bash
cd ~/ROS2_Autonomous_2026_27
git fetch origin
git push origin sohan/workspace-colcon-build-fix
git push origin sohan/rover_slam-rtabmap-bringup
```

Open both PRs against `main`. CONTRIBUTING.md section 3 requires shared-area changes to be
called out, and the SLAM branch has two:

- `src/rover_gazebo/urdf/rover_sensors.xacro`, one line, another owner's package. Section 2
  above is the justification.
- the root `README.md`, package status only.

The SLAM branch sits on top of `pradyun/rover_gazebo-urdf-sim-bringup` and cannot merge
before that branch does. It also carries the build fix as a merge, so whichever lands first
the other stays clean.

One thing to hand to Pradyun rather than fix here: none of the three worlds has a surface
material, which is why `odom_source:=visual` cannot work. A ground texture would make the
camera path usable and make the simulation a fairer stand-in for the real rover.
