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

All eleven are currently **placeholder directories** — each holds only a README stating its scope
and owner. Build files are added by each owner in their first implementation PR.

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

Until a package has `package.xml`/`CMakeLists.txt` files, `rosdep` and `colcon build` are no-ops
for it — they will succeed but build nothing. [`rover_drivers`](src/rover_drivers/) is the first
package with real build files; see its README for driver-specific prerequisites (e.g. the ZED SDK
is a binary install, not something `rosdep` can fetch). The intended way to run all of this is
inside the container defined in [`docker/`](docker/).

## Contributing

Everyone works on their own branch, normally inside their own `src/<package>/`, and merges into
`main` via pull request. Read [CONTRIBUTING.md](CONTRIBUTING.md) before your first PR.
