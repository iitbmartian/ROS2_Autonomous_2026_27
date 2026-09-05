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
                                        /map  /rtabmap/cloud_map  TF map→odom
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
ros2 run rover_gazebo teleop_rover.py     # another terminal
```

Mapping over the obstacle courses:

```bash
ros2 launch rover_slam slam_sim.launch.py world:=ledge.sdf
ros2 launch rover_slam slam_sim.launch.py world:=bars.sdf
```

Localise against a map you already built:

```bash
ros2 launch rover_slam slam_sim.launch.py localization:=true
```

Take the estimator out of the loop, to tell a mapping fault from an odometry fault:

```bash
ros2 launch rover_slam slam_sim.launch.py odom_source:=ground_truth
```

---

## 7. What was verified, and what was not

Verified here:

- `colcon build` clean for both packages.
- The processed xacro carries `<gz_frame_id>camera_optical_frame</gz_frame_id>`, so the frame
  fix reaches the model.
- Both launch branches start clean with no simulator running: every node reaches a steady
  state waiting for topics, and neither logs a single warning. The mapper confirms it loaded
  `Grid/Sensor=2`, `Grid/3D=false`, `RGBD/CreateOccupancyGrid=true`, `Reg/Strategy=2`, and in
  localisation mode `Mem/IncrementalMemory=false`.

**Not verified here**, because it needs a GPU and several minutes of driving:

1. `ros2 topic echo /camera/camera_info --once` reports `frame_id: camera_optical_frame`.
2. `ros2 run tf2_tools view_frames` shows `map` → `odom` → `base_footprint` → `base_link`,
   with `map` → `odom` owned by `rtabmap` and `odom` → `base_footprint` by `rgbd_odometry`.
3. `ros2 topic hz /map` and `ros2 topic hz /rtabmap/cloud_map` both publish, and the grid
   grows in RViz as you drive.
4. Drive a circuit on `bars.sdf`, return to the start, and `ros2 topic echo /rtabmap/info`
   shows a non-zero `loopClosureId`.
5. `odom_source:=ground_truth` still maps.

Run those five before trusting the map. Items 1 and 2 are the ones that catch a frame
problem, and they take seconds.

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

---

## 9. Pushing this

The branch is `sohan/rover_slam-rtabmap-bringup`, committed and ready:

```bash
cd ~/ROS2_Autonomous_2026_27
git fetch origin
git push -u origin sohan/rover_slam-rtabmap-bringup
```

Open the PR against `main`. CONTRIBUTING.md section 3 requires shared-area changes to be
called out explicitly, and this branch has two:

- `src/rover_gazebo/urdf/rover_sensors.xacro`, one line, another owner's package. Section 2
  above is the justification.
- the root `README.md`, package status only.

The PR also needs to say that it sits on top of `pradyun/rover_gazebo-urdf-sim-bringup` and
cannot merge before that branch does.
