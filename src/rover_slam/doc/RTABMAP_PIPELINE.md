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

This is the only change to Pradyun's package, and it needs calling out in the PR. It is worth
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
| `launch/rtabmap.launch.py` | the SLAM stack; attaches to an already-running simulation |
| `launch/slam_sim.launch.py` | simulation and SLAM in one command |
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
| `Grid/3D` | `false` | this is the squish: project the cloud onto xy and emit a plain `OccupancyGrid` |
| `Grid/RayTracing` | `true` | fill known-free space between the rover and each hit, instead of leaving it unknown |
| `Grid/NormalsSegmentation` | `false` | height thresholds instead of surface normals, see below |
| `Grid/MaxGroundHeight` | `0.15` | clears the 10 cm ledge in `ledge.sdf` |
| `Grid/MaxObstacleHeight` | `1.0` | ignore anything the rover can drive under |
| `Grid/Footprint*` | 1.0 × 0.8 × 0.5 | do not map the rover's own wheels as obstacles |

The normals-versus-height choice matters for this rover specifically. `ledge.sdf` is a 10 cm
step the suspension is designed to climb, and a normals-based segmenter reads its vertical
face as a wall. Height thresholding lets the rover plan over what it can actually drive over.

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
colcon build --symlink-install --packages-select rover_gazebo rover_slam
source install/setup.bash
ros2 launch rover_slam slam_sim.launch.py rviz:=true
ros2 run rover_gazebo teleop_rover.py     # another terminal, keep it focused
```

`W`/`S` drive, `A`/`D` turn, `1`/`2`/`3` pick Ackermann, crab or spot turn, space stops.
These are not the keys `teleop_twist_keyboard` sends, which is the usual reason the rover
sits still. The map only grows while the rover moves; watch `WM=` climb in the SLAM log.

In RViz set Fixed Frame to `map`. The grey and black grid is `/map`, the coloured points
are `/cloud_map`.

### Choosing the odometry source

This matters more than anything else in the file, because every world in `rover_gazebo` is
a bare ground plane plus at most two boxes.

| World | Works with | Why |
|---|---|---|
| `flat.sdf` | `ground_truth` only | no texture for vision, no geometry for ICP |
| `bars.sdf` | `ground_truth`, `lidar` | two bars give ICP something to bite on |
| `ledge.sdf` | `ground_truth`, `lidar` | ramp and step, likewise |
| `husarion_world.sdf` | `ground_truth` only | a grey plane and a floor decal, so `flat.sdf` in substance |

`ground_truth` is the default. It takes the simulator's own pose, so a mapping fault can be
told apart from an estimator fault, and it is the only source that works on `flat.sdf`.

`lidar` runs `rtabmap_odom/icp_odometry` on `/lidar/points`. It is the honest test, since
it uses a sensor the real rover carries:

```bash
ros2 launch rover_slam slam_sim.launch.py world:=bars.sdf odom_source:=lidar rviz:=true
```

`visual` runs `rgbd_odometry`. It has never initialised in any world here: it needs 15
tracked features and finds about 8 against blank surfaces, logging `15 visual features
required to initialize the odometry` forever. Downstream the mapper then rejects every
frame with `no odometry is provided. Image 0 is ignored!`. Nothing is wrong with the
settings, the worlds simply have no texture. Adding a ground material to the worlds is a
`rover_gazebo` change and would make this path usable.

Localise against a map you already built:

```bash
ros2 launch rover_slam slam_sim.launch.py localization:=true
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

**Still not verified:**

1. **Loop closure.** Needs a circuit driven back to its own start, then
   `ros2 topic echo /info` showing a non-zero `loopClosureId`, or the loop
   closure counter climbing in `rtabmap_viz`.
2. **Localization mode** against a database saved from a previous run.
3. **`odom_source:=visual`**, which cannot be tested until a world has texture in it.

---

## 8. When it misbehaves

**`/map` never appears.** Check `RGBD/CreateOccupancyGrid` is `true`. It is off by default and
fails silently.

**Nothing arrives at all, no errors.** QoS mismatch. `qos` must be 1 against the bridge.

**The cloud is on its side, or depth lands in the wrong place.** The camera frame fix has been
lost. `ros2 topic echo /camera/camera_info --once` must say `camera_optical_frame`.

**"Did not receive data" warnings.** Timestamps are not lining up. Check `use_sim_time` is
true everywhere, that `/clock` is being published, and widen `approx_sync_max_interval`.

**Odometry keeps resetting.** The rover is probably in crab or spot mode, which look nothing
like the visual motion model. `Odom/ResetCountdown` is 10, so it recovers rather than dying.
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
