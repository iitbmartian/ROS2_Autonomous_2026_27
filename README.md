# ROS2_Autonomous_2026_27

Autonomous subsystem for the IITB Mars Rover Team, 2026-27 season. Successor to
`ROS2_Autonomous_Subdivision`.

This repository is a colcon workspace: cloning it gives you the workspace directly, with all
packages under [`src/`](src/).


## Packages

| Package | Owner(s) | Scope |
|---|---|---|
| [`rover_interfaces`](src/rover_interfaces/) | (unassigned) | pub/sub, server-client, action, custom messages |
| [`rover_perception`](src/rover_perception/) | Apratim + Pradyun | object detection, object classification, AR tag detection |
| [`rover_nav2`](src/rover_nav2/) | Ram + Apratim | path planning, control, tuning, 3D navigation |
| [`rover_slam`](src/rover_slam/) | Sohan (VO) | 3D SLAM, RTAB-Map, 3D→2D squishing, loop closures |
| [`rover_strategy`](src/rover_strategy/) | Nishant + Sohan | exploration, task allocation, recovery |
| [`rover_drivers`](src/rover_drivers/) | Ram | zed2i, 4D lidar, 2D lidar, sbg |
| [`rover_ekf`](src/rover_ekf/) | Nishant + Sohan | 4x explicit + 4x drive → KF; 4 KF + sbg + VO + IMU(zed2) → EKF adaptive matrices |
| [`rover_gazebo`](src/rover_gazebo/) | Pradyun | URDF, high-quality Gazebo environments |
| [`rover_unity`](src/rover_unity/) | Ram + Apratim | Unity simulation environments |
| [`rover_bringup`](src/rover_bringup/) | (unassigned) | launch files, config files, bashfile |
| [`rover_controls`](src/rover_controls/) | (unassigned) | input: wheel encoders, IMU, etc → output: ROS2 odometry topic |

`rover_gazebo` and `rover_slam` are implemented. Eight of the rest are **placeholder
directories** — each holds only a README stating its scope and owner. Build files are added by
each owner in their first implementation PR.

`rover_slam` runs RTAB-Map against the `rover_gazebo` simulation, and
[`src/rover_slam/doc/RTABMAP_PIPELINE.md`](src/rover_slam/doc/RTABMAP_PIPELINE.md) documents
how the two fit together.

`rover_drivers` is the exception: it carries vendored upstream driver sources for the SBG, ZED
2i and Unitree 4D lidar hardware. Not all of them build on a stock machine — see below.

The architecture diagram shows Gazebo and Unity as one node; they are split into two packages here
because they have different owners and very different dependency trees.

## Quickstart

```bash
git clone https://github.com/iitbmartian/ROS2_Autonomous_2026_27.git
cd ROS2_Autonomous_2026_27
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`rosdep` and `colcon build` build `rover_gazebo`, `rover_slam` and, of the vendored drivers,
`sbg_driver` and `zed_msgs`. The placeholder packages have no `package.xml` and are skipped.

Two caveats on a fresh clone:

- **The ZED wrapper needs the proprietary [ZED SDK](https://www.stereolabs.com/developers/release)**
  and CUDA. `zed_components` and `zed_debug` both call `find_package(ZED)` and fail without it;
  `zed_wrapper` and `zed_ros2` are then skipped as dependants. On a machine with no ZED hardware,
  drop a local `COLCON_IGNORE` into `src/rover_drivers/zed2i_ws/src/zed-ros2-wrapper/` to skip all
  four. That filename is gitignored, so it stays on your machine, and `zed_msgs` in
  `zed-ros2-interfaces` still builds.
- **Four Unitree lidar trees are skipped by tracked `COLCON_IGNORE` files.** Two are ROS 1 catkin
  packages that cannot build under Jazzy at all. The other two are blocked on a prebuilt archive,
  `unitree_lidar_sdk/lib/x86_64/libunilidar_sdk2.a`, that was never committed. Each marker file
  explains its own case.

Add `--continue-on-error` to see every failure at once; without it the first one aborts the run.
The intended way to run all of this is inside the container defined in [`docker/`](docker/).

## Contributing

Everyone works on their own branch, normally inside their own `src/<package>/`, and merges into
`main` via pull request. Read [CONTRIBUTING.md](CONTRIBUTING.md) before your first PR.
