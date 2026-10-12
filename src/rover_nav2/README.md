# rover_nav2

**Owner(s):** Ram + Apratim  
**Status:** in progress — Nav2 params and launch files, ported from the team's 2-D robot workspace

## Scope

- Path planning
- Control
- Tuning
- 3D navigation

## What this package is

Nav2 configuration and launch, nothing else: this package owns the parameters, the plugin
selection and the tuning. It starts no drivers, no SLAM and no perception. Those come from
[`rover_drivers`](../rover_drivers/), [`rover_slam`](../rover_slam/) and
[`rover_ekf`](../rover_ekf/), composed by [`rover_bringup`](../rover_bringup/).

The tuning is inherited from the earlier 2-D navigation workspace (`slam_mrt-jazzy`,
package `rover_gazebosim`) and re-targeted at the rover. The parameter files are written
against **Nav2 1.3.x (ROS 2 Jazzy)**, the version `rosdep` installs for this workspace. Keys the
old workspace carried that only exist in newer Nav2 releases were dropped rather than left in
place, because Jazzy ignores unknown parameters silently and that hides real typos. The dropped
ones are `path_handler_plugins` / `FeasiblePathHandler`, the MPPI `TrajectoryValidator` block and
`publish_optimal_trajectory`, `bt_search_directories`, `error_code_name_prefixes`,
`filter_duration`, `search_window`, `introspection_mode`, `allow_partial_planning`,
`path_length_tolerance`, `random_seed`, the per-behaviour acceleration limits, and the
`mppi::DiffDriveMotionModel` plugin line (in Jazzy the motion model is the `motion_model` string
alone).

## Interfaces

Everything below is a contract with another package. Changing a name here means changing it
there too.

| Direction | Topic / TF | Type | Peer |
|---|---|---|---|
| in | `/map` | `nav_msgs/OccupancyGrid` | `rover_slam` — 3D SLAM squished to a 2D grid |
| in | `map` → `odom` | TF | `rover_slam` |
| in | `odom` → `base_link` | TF | `rover_ekf` |
| in | `/odometry/filtered` | `nav_msgs/Odometry` | `rover_ekf` (robot_localization's default topic) |
| in | `/obstacle_cloud` | `sensor_msgs/PointCloud2` | fused lidar + camera cloud, clustered |
| out | `/cmd_vel` | `geometry_msgs/TwistStamped` | `rover_controls` |

`/obstacle_cloud` is the only obstacle source: the local costmap voxelises it, the global costmap
marks it over the static map, and the collision monitor checks it against the footprint. Points
below 0.12 m are treated as ground, points above 1.6 m as overhead clearance. If the fusion node
publishes under a different name, change `topic:` in the three `fused_cloud` blocks of
`config/nav2_params.yaml` — global costmap, local costmap, collision monitor. That name is
provisional; settle it with whoever owns the fusion node.

Velocity leaves Nav2 through a chain, not a single node: the controller and the recovery
behaviours publish `/cmd_vel_nav`, the velocity smoother republishes `/cmd_vel_smoothed`, and the
collision monitor publishes the final `/cmd_vel`. Every one of them has
`enable_stamped_cmd_vel: true`, so the whole chain is `TwistStamped`. No relay node is involved,
unlike the old workspace.

Three nodes read `odom_topic` independently and all three are pointed at
`/odometry/filtered`: `bt_navigator`, `velocity_smoother`, and `controller_server`'s own internal
odometry smoother (a separate parameter from `bt_navigator`'s, easy to miss — Nav2's default for
it is `odom`, not the value set anywhere else in the file). If a fourth node grows the same need,
check it explicitly rather than assuming the default matches.

## Layout

| Path | Purpose |
|---|---|
| `config/nav2_params.yaml` | Real rover. The file to tune. |
| `config/nav2_params_sim.yaml` | Gazebo. Same structure, retargeted at `rover_gazebo` + `rover_slam`: see [Simulation](#simulation). |
| `behavior_trees/*_ackermann.xml` | Jazzy's default navigate-to-pose / through-poses trees minus the Spin recovery. Used by the sim params. |
| `rviz/nav2_sim.rviz` | Map, costmaps, plans, footprint, lidar, and the "Nav2 Goal" tool. |
| `rover_bringup/launch/navigation.launch.py` | The Nav2 stack: planner, controller, smoother, behaviours, BT navigator, route server, waypoint follower, velocity smoother, collision monitor, lifecycle manager. No docking server (see Plugin choices). |
| `rover_bringup/launch/pipeline.launch.py` | The whole sim stack: Gazebo, the EKF, RTAB-Map, a depth-image obstacle cloud and `navigation.launch.py sim:=true`. `nav_rviz:=true` opens `rviz/nav2_sim.rviz`. |

The launch file builds its own node list rather than including `nav2_bringup`'s. That package
is a demo/tutorial bundle whose `package.xml` hard-depends on Gazebo, RViz and `slam_toolbox` —
none of which this rover needs — so it is deliberately not a dependency of `rover_nav2` at all.
The `navigation2` metapackage alone (declared in `package.xml`) covers every server and plugin
used here.

## Running

```bash
rosdep install --from-paths src --ignore-src -r -y     # installs Nav2 itself
colcon build --symlink-install --packages-select rover_nav2
source install/setup.bash

ros2 launch rover_bringup navigation.launch.py            # real rover
ros2 launch rover_bringup navigation.launch.py sim:=true  # simulator, Nav2 only
ros2 launch rover_bringup pipeline.launch.py nav_rviz:=true  # Gazebo + EKF + RTAB-Map + Nav2 + RViz
```

`sim:=true` picks the sim parameter file and the simulated clock together. Override either on its
own with `params_file:=...` or `use_sim_time:=...`. The other arguments are `namespace` and
`autostart`.

Launched alone, Nav2 activates and then waits: the costmaps stay empty until `rover_slam`
publishes `/map` and the fusion node publishes `/obstacle_cloud`. That is expected, not a fault.

Localisation comes only from RTAB-Map (`rover_bringup/launch/rtabmap.launch.py`); there is no
AMCL path. To drive a course mapped earlier, run RTAB-Map against its saved database:

```bash
ros2 launch rover_bringup pipeline.launch.py localization:=true
```

## Simulation

`rover_bringup`'s `pipeline.launch.py` runs the whole stack against `rover_gazebo`:

```bash
ros2 launch rover_bringup pipeline.launch.py nav_rviz:=true                       # empty_world.sdf, EKF odom
ros2 launch rover_bringup pipeline.launch.py world:=flat.sdf odom_source:=ground_truth nav_rviz:=true
ros2 launch rover_bringup pipeline.launch.py world:=marsyard.sdf odom_source:=visual nav_rviz:=true
```

It starts Nav2 15 s in, once `/map` and `map` → `odom` exist. Send goals with **Nav2 Goal** in RViz.
Where `map` sits depends on `odom_source`. Under `ground_truth` (the default), odometry comes
straight from Gazebo, so `map` is the Gazebo world frame and a rover spawned with `x:=-13` sits at
(−13, 0). Under `lidar` or `visual`, `map` starts wherever the rover spawned.
In `husarion_world.sdf`, spawn with `x:=-13` and send goals west, north or south. The Husarion
logo is a solid plate 1.8 m tall that spans x −11.96…12.66, directly in front of the rover. Any
`x` above about −12.6 (for example `x:=-10`) puts part of the rover inside it.
Goals have to lie inside the map RTAB-Map has built so far. The global costmap is sized to
`/map`, and Smac rejects a goal outside it with "Goal Coordinates ... was outside bounds".

What the sim wiring depends on, and why `nav2_params_sim.yaml` differs from the real file:

- **`/cmd_vel` is `TwistStamped` end to end.** `rover_gazebo`'s `rover_kinematics` subscribes to
  `TwistStamped`, as do its teleop and measurement scripts. A `Twist` publisher and a
  `TwistStamped` subscriber on one topic never connect, and nothing logs it.
- **Frames: `base_footprint`, not `base_link`.** `odom` → `base_footprint` comes from the EKF (or the chosen `odom_source`),
  and `rover_gazebo`'s `base_link` sits 0.7225 m up at the chassis origin. Obstacle heights in the
  collision monitor are measured in its base frame, so it has to be the one on the ground.
- **Obstacles come from sensors that exist.** `/lidar/points` feeds both costmaps and the
  collision monitor. `/camera/depth_points` feeds the local costmap; `pipeline.launch.py` builds it
  from the depth image with `rtabmap_util/point_cloud_xyz`. Every source keeps points 0.15–1.0 m
  above the ground, which matches `rover_slam`'s `Grid/MaxGroundHeight` and
  `Grid/MaxObstacleHeight`.
  **Do not use `rover_gazebo`'s `/camera/points`**: Gazebo publishes that cloud in the camera's
  body axes but stamps it `camera_optical_frame`, so the ground ahead shows up as a phantom wall
  beside the rover and pins it in place.
- **Ackermann.** `rover_kinematics` runs in ackermann mode, which caps yaw rate at
  |vx| / 0.8 m, so a spin in place produces no motion. MPPI therefore uses `motion_model: Ackermann`
  with `min_turning_r: 0.8`, the planner is Smac hybrid-A* (Reeds-Shepp, 0.8 m radius), and the
  behaviour trees leave out Spin.
- **Inflation radii** (0.85 m local, 1.0 m global) are above the footprint's 0.78 m circumscribed
  radius. Below that, Smac and MPPI fall back to slow full-footprint checks.
- **MPPI checks the whole footprint** (`CostCritic.consider_footprint: true`). Checking only
  the centre point let it choose trajectories whose corners clipped walls.
- **`wait_for_service_timeout` is 5 s**, and `default_server_timeout` is 100 ms. At the
  defaults (1 s and 20 ms), a planner_server still busy starting Smac could make bt_navigator
  abort the whole bringup, or abort a goal before planning began.
- **`/odom` is Gazebo ground truth.** Nav2 reads only its twist; the pose comes from TF, so this
  holds for every `odom_source`.

Verified on 2026-09-24 (Jazzy, Gazebo Harmonic, headless). With `world:=flat.sdf
odom_source:=ground_truth`, two goals were sent one after the other from the spawn point:
(5, 1.5) and then (7, −2). Both `SUCCEEDED`, in 26 s and 22 s, with zero progress-checker
failures, and the ground-truth pose finished within 8 cm of each goal.

Tuning pass, same day. Ground truth was recorded at 2 Hz during each run, with sideways slip
taken from `/odom`'s body-frame twist.

- **Sideways "crabbing" came from `rover_kinematics`, not Nav2.** At every change between forward
  and reverse, which Ackermann three-point turns do all the time, the wheels swept through
  sideways (up to 89°), and the rover slid 40–50° off its heading. That's fixed in
  `rover_gazebo/rover_kinematics_node.py` (see its docstring). Afterwards, sideways slip above
  15° appeared only momentarily, at near-zero speed while reversing: 4 of 197 samples in the
  husarion run below, 1 of 203 in marsyard.
- **husarion_world.sdf, `x:=-13`.** Three legs in sequence: a three-point turn off the plate
  with the nose 0.5 m from it, 7 m north along the wall, then a right turn around the plate's
  corner to (−9, 14.3). All `SUCCEEDED` (46, 32 and 22 s), with 0 progress failures and 0
  recoveries.
- **marsyard.sdf.** Three goals through the rock field: (7.5, 0.5), (5, 6.5) and (−2, 7). All
  `SUCCEEDED` (37, 39 and 33 s), with 0 recoveries. The closest the wheels came to any of the 19
  rocks taller than 0.15 m was 0.73 m (median 2.83 m).

Known limits, not fixed here:

- **bars.sdf.** The rover plans and drives, and once past the first bar it reached (10, 2). Coming
  from the spawn point, though, it hit the 8 cm bar at an angle at crawl speed and stalled with a
  front wheel on the bar's corner. The same bars are climbed at 1.0 m/s in `rover_gazebo`'s
  VERIFICATION.md.
- **`odom_source:=lidar`** diverged in both bars.sdf and husarion_world.sdf while Nav2 drove.
  icp_odometry logged "structural complexity is too low" hundreds of times, and the odom frame
  drifted tens of metres from ground truth. That is a limit of `rover_slam`'s odometry in these
  sparse worlds, not of the Nav2 wiring.

## Plugin choices

Real rover (`nav2_params.yaml`). The simulator differs where the section above says so.

- **Controller: MPPI** (`nav2_mppi_controller`), differential-drive motion model. Six-wheel skid
  steer is differential drive as far as Nav2 is concerned. The critic weights carry over from the
  2-D robot; the velocity and acceleration limits do not, and are set low for a first run on
  hardware.
- **Planner: NavFn** (Dijkstra, `use_astar: false`). Adequate for a differential-drive base on a
  2D grid. If the rover needs curvature-aware plans over open terrain, Smac's hybrid-A* drops into
  the `GridBased` block.
- **Behaviours:** spin, back up, drive on heading, wait.
- **Collision monitor** is the last link in the velocity chain, so it stays running. Confirmed by
  test (see below): if `/obstacle_cloud` has no publisher, or stops publishing for longer than
  `source_timeout` (1 s), collision monitor treats the source as invalid and **zeroes `/cmd_vel`
  outright** — the rover stops, it does not drive blind. That is the fail-safe behaviour and is
  the reason a path can be planned and `/cmd_vel_nav` can carry a real MPPI velocity while the
  final `/cmd_vel` stays at zero: it means the obstacle-cloud source is missing or stale, not that
  the controller failed.
- **Costmap filters** (keepout, speed) are configured but disabled, and no filter-mask server is
  launched. Enable them once a mask for the arena exists.
- **Docking** is not launched. Nothing on the rover docks, and `opennav_docking` 1.3.x
  intermittently segfaults while configuring (seen 1 in 8 sim runs). When it does, the
  lifecycle manager leaves every other Nav2 node `inactive`. The `docking_server` parameter blocks
  stay in both files, so adding the node back to `navigation.launch.py` is all it takes.

## How this was verified

Neither `rover_slam` nor `rover_ekf` exist yet, so this package was tested with throwaway
stand-ins for their outputs, not against the real pipeline:

- `nav2_map_server` serving a blank 10x10 m test map, in place of `rover_slam`'s `/map`.
- `nav2_loopback_sim` (a Nav2-provided package, `sudo apt install ros-jazzy-nav2-loopback-sim`),
  in place of `rover_ekf`: it turns `/cmd_vel` into fake odometry and publishes `odom`→`base_link`
  and, once given a `/initialpose` (as "2D Pose Estimate" in RViz would), `map`→`odom` TF.
- A ten-line script publishing an empty `PointCloud2` on `/obstacle_cloud` at 10 Hz, in place of
  the fusion node — enough to satisfy `source_timeout`, not enough to represent a real obstacle.

None of that scaffolding lives in this repo; it was run from `/tmp` and discarded. With it in
place: `ros2 launch rover_bringup navigation.launch.py`, all ten lifecycle nodes reached `active`
with no errors, every MPPI critic and costmap plugin loaded with the configured weights, and a
`NavigateToPose` goal 2 m ahead planned, drove (`/cmd_vel` peaked around 0.18 m/s, under the 0.5
m/s limit), and completed with `SUCCEEDED` and zero recoveries, stopping within the configured
0.30 m `xy_goal_tolerance`.

Two real issues surfaced this way and are already fixed in this package, not just noted:
`controller_server`'s own `odom_topic` parameter (separate from `bt_navigator`'s, see Interfaces
above) was missing and would have silently defaulted to `odom` instead of `/odometry/filtered`;
and `nav2_bringup` was pulling ~230 packages including Gazebo and RViz that this package has no
use for, fixed by owning the node lists directly (see Layout above) instead of including its
launch files.

**If you want to rerun this:** on a machine without `~/.local`'s NumPy shadowing the system one,
skip `PYTHONNOUSERSITE=1`; here it was needed because `nav2_loopback_sim`'s `transforms3d`
dependency breaks under NumPy 2.x. Full sequence: bring up the map server and loopback sim,
publish an initial pose once TF is flowing, start `publish_empty_cloud.py`-equivalent, launch
`navigation.launch.py`, then `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose
"{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
--feedback`.

This proves the planner, controller, and costmap tuning in this package. It does not prove
anything about `rover_slam`'s map quality, `rover_ekf`'s real odometry accuracy, or the real
fusion node — those need the full stack once it exists.

## Open items

- **Footprint** is a 1.1 m x 0.9 m box inherited from the 2-D robot. Confirm it against the rover
  URDF once [`rover_gazebo`](../rover_gazebo/) lands, and update both costmaps in both parameter
  files. The inflation radii (0.85 m local, 1.0 m global) follow from it.
- **Velocity limits** (0.5 m/s forward, 1.0 rad/s) are deliberately low. Raise them once the drive
  train and the EKF have been checked on rough ground. They appear twice: in the MPPI block and in
  the velocity smoother.
- **`/obstacle_cloud`** is a placeholder name until the fusion node exists.
- **License** is still `TODO: License declaration` in `package.xml`, matching the rest of the
  workspace.

See [CONTRIBUTING.md](../../CONTRIBUTING.md) for the branch and PR workflow.
