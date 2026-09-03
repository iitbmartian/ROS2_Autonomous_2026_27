# rover_gazebo

**Owner(s):** Pradyun  
**Status:** in progress — URDF, meshes, and Gazebo bringup added; `ament_cmake` build type

## Scope

- URDF / robot description
- High-quality Gazebo environments

## Notes

The architecture diagram shows Gazebo and Unity as a single node, but they have different owners
and very different dependency trees, so they are split into two packages. Unity work lives in
[`rover_unity`](../rover_unity/).

## Getting started

```bash
colcon build --packages-select rover_gazebo --symlink-install
source install/setup.bash
./src/rover_gazebo/run_sim.sh
```

`run_sim.sh` launches Gazebo with the rover URDF spawned in `worlds/khali.sdf`, bridged to ROS 2
topics via `config/ros_gz_bridge.yaml`. For a lighter RViz-only view of the URDF, use
`ros2 launch rover_gazebo display.launch.py` instead.

See [CONTRIBUTING.md](../../CONTRIBUTING.md) for the branch and PR workflow.
