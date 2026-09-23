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

`rover_interfaces`, `rover_gazebo`, `rover_ekf`, and `rover_strategy` have build files as of their
respective implementation PRs; the remaining seven are still **placeholder directories** — each
holds only a README stating its scope and owner. Build files are added by each owner in their
first implementation PR.

The architecture diagram shows Gazebo and Unity as one node; they are split into two packages here
because they have different owners and very different dependency trees.

## Requirements

Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic. Harmonic comes from the ROS vendor packages
(`ros-jazzy-gz-sim-vendor` and friends), which `rosdep` pulls in through `ros_gz_sim`; there is no
separate `gz-harmonic` install to do.

## Quickstart

```bash
git clone https://github.com/iitbmartian/ROS2_Autonomous_2026_27.git
cd ROS2_Autonomous_2026_27

export HUSARION_ROS_BUILD_TYPE=simulation   # required, see below
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
```

Until a package has `package.xml`/`CMakeLists.txt` files, `rosdep` and `colcon build` are no-ops
for it — they will succeed but build nothing. [`rover_drivers`](src/rover_drivers/) is the first
package with real build files; see its README for driver-specific prerequisites (e.g. the ZED SDK
is a binary install, not something `rosdep` can fetch). The intended way to run all of this is
inside the container defined in [`docker/`](docker/).
`HUSARION_ROS_BUILD_TYPE=simulation` is not optional. The two Husarion description packages guard
their simulation dependencies behind that variable in `package.xml`. Left unset, the condition is
false and `rosdep` silently skips `ros_gz_sim`, `ros_gz_bridge` and `rviz2`, and the build appears
to succeed right up until Gazebo fails to launch. Export it in your shell profile.

## Running the simulation

```bash
ros2 launch rover_gazebosim spawn_rover.launch.py                 # Gazebo + rover only
ros2 launch rover_gazebosim pipeline_launch.launch.py             # + EKF, SLAM, nav2, mission
```

`spawn_rover.launch.py` takes a `world:=` argument. It defaults to `husarion_world.sdf`; the other
worlds in `rover_gazebosim/world/`, and Gazebo's own `empty.sdf`, also work.

The intended way to run all of this is inside the container defined in [`docker/`](docker/).

## Contributing

Everyone works on their own branch, normally inside their own `src/<package>/`, and merges into
`main` via pull request. Read [CONTRIBUTING.md](CONTRIBUTING.md) before your first PR.
