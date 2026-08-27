# rover_unity

**Owner(s):** Ram + Apratim  
**Status:** placeholder — no implementation yet

## Scope

- Unity simulation environments
- ROS2 ↔ Unity bridge

## Notes

The architecture diagram shows Gazebo and Unity as a single node, but they have different owners
and very different dependency trees, so they are split into two packages. Gazebo work and the URDF
live in [`rover_gazebo`](../rover_gazebo/).

## Getting started

Build files (`package.xml`, and `CMakeLists.txt` or `setup.py`) are to be added by the
owner in the first implementation PR — the build type (`ament_cmake` vs `ament_python`)
is the owner's call.

See [CONTRIBUTING.md](../../CONTRIBUTING.md) for the branch and PR workflow.
