# Local changes to vendored code

Every change to files under `third_party/` is listed here, so an update to a new upstream commit
can reapply them. Each patched spot is also marked `rover_terrain_mapping PATCH <n>` in the file.

1. `fastdem/CMakeLists.txt`, nanoGrid lookup. Upstream fetches nanoGrid with FetchContent at
   `GIT_TAG main`, which is unpinned and needs network at build time. Patched to use the
   `nanoGrid::nanoGrid` target that `rover_terrain_mapping` defines from `third_party/nanogrid`,
   falling back to `find_package(nanoGrid REQUIRED)` when built standalone.
2. `fastdem/CMakeLists.txt`, install rules. Wrapped in a top-level check, since vendored FastDEM is
   linked statically into this package's executables and nothing of it is installed.

Not patched, handled from `rover_terrain_mapping/CMakeLists.txt` instead:

- FastDEM's `tests/CMakeLists.txt` fetches googletest with FetchContent. It is not used. The
  unchanged test sources are registered with `ament_add_gtest` against the system gtest.
- `examples/01_hello_mapping` writes its image next to its source. It is built with
  `EXAMPLE_OUTPUT_DIR` pointing into the build tree.
- Upstream compiler warnings are silenced with `-w` on the vendored targets only.

Upstream tests only on ROS 2 Humble. On Jazzy (Ubuntu 24.04, GCC 13, Eigen 3.4, yaml-cpp 0.8,
spdlog 1.12) the core built and all its tests passed with no source change.
