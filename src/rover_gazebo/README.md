# rover_gazebo

**Owner(s):** Pradyun  
**Status:** placeholder — no implementation yet

## Scope

- URDF / robot description
- High-quality Gazebo environments

## Notes

The architecture diagram shows Gazebo and Unity as a single node, but they have different owners
and very different dependency trees, so they are split into two packages. Unity work lives in
[`rover_unity`](../rover_unity/).

## Getting started

Build files (`package.xml`, and `CMakeLists.txt` or `setup.py`) are to be added by the
owner in the first implementation PR — the build type (`ament_cmake` vs `ament_python`)
is the owner's call.

See [CONTRIBUTING.md](../../CONTRIBUTING.md) for the branch and PR workflow.
