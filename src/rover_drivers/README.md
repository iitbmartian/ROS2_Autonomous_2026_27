# rover_drivers

**Owner(s):** Ram
**Status:** in progress — sbg, 4D lidar, and zed2i vendored and buildable

## Scope

- zed2i
- 4D lidar
- 2D lidar
- sbg

## Layout

| Folder | Package(s) | Notes |
|---|---|---|
| [`sbg_ros2/`](sbg_ros2/) | `sbg_driver` | Vendors the sbgECom C library under `external/`; builds standalone. |
| [`unitree_4d_lidar_l2/`](unitree_4d_lidar_l2/) | `unitree_lidar_ros2` | Unitree's official Unilidar L2 SDK (`unilidar_sdk2-2.0.10/`). Only `unitree_lidar_ros2` is ROS2/ament — `unitree_lidar_ros` (ROS1 catkin) and the sibling `point_lio_unilidar-2.0.2/` (ROS1 catkin, SLAM front-end) are marked `COLCON_IGNORE` since this repo is ROS2-only and `catkin` isn't installed here. They're kept for reference/future porting. |
| [`zed2i_ws/`](zed2i_ws/) | `zed_msgs`, `zed_ros2`, `zed_components`, `zed_wrapper` | Stereolabs' `zed-ros2-interfaces` + `zed-ros2-wrapper`, vendored as plain tracked files (not submodules). `zed2i_ws/src/zed-ros2-examples/` is present locally for reference but marked `COLCON_IGNORE` and untracked — it pulls in extra deps (rviz plugins, Isaac ROS demos) not needed for the driver itself. |

## Build prerequisites

Everything except the ZED wrapper builds from vendored source with plain `rosdep`. The ZED2i
driver additionally needs, installed as system binaries (not managed by `rosdep`/`colcon`):

- **ZED SDK** (v5.2, matching your CUDA version) — <https://www.stereolabs.com/developers/release>
- **CUDA Toolkit** — <https://developer.nvidia.com/cuda-downloads>

Both are required by `find_package(ZED REQUIRED)` / `find_package(CUDAToolkit REQUIRED)` in
`zed2i_ws/src/zed-ros2-wrapper/zed_components/CMakeLists.txt`.

## Build

From the repo root:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

This builds `sbg_driver`, `unitree_lidar_ros2`, and the ZED2i stack
(`zed_msgs`, `zed_ros2`, `zed_components`, `zed_wrapper`) together with the rest of the workspace.
`zed_debug` is dev-only upstream and can be dropped with `--packages-skip zed_debug` if you don't
need it.

See [CONTRIBUTING.md](../../CONTRIBUTING.md) for the branch and PR workflow.
