cmake_minimum_required(VERSION 3.16)
project(rover_terrain_mapping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  # FastDEM's per-scan cost assumes an optimised build.
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)

# ---------------------------------------------------------------------------
# Vendored FastDEM, built as static libraries and linked into this package's
# executables. See third_party/VERSION and third_party/PATCHES.md.
# ---------------------------------------------------------------------------
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_subdirectory(third_party/nanogrid EXCLUDE_FROM_ALL)
add_subdirectory(third_party/fastdem EXCLUDE_FROM_ALL)
# Upstream warnings are not ours to fix.
foreach(t fastdem nanoGrid)
  target_compile_options(${t} PRIVATE -w)
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # FastDEM's own suite, unchanged sources, registered with the system gtest instead of its
  # FetchContent googletest.
  set(fastdem_tests
    test_config test_elevation_map test_rasterization test_kalman_estimation
    test_quantile_estimation test_dual_layer test_sensor_models test_postprocess
    test_fastdem_integration test_online_mode test_map_io)
  foreach(t ${fastdem_tests})
    ament_add_gtest(fastdem_${t} third_party/fastdem/tests/${t}.cpp)
    target_link_libraries(fastdem_${t} fastdem)
    target_compile_options(fastdem_${t} PRIVATE -w)
  endforeach()
  target_compile_definitions(fastdem_test_config PRIVATE
    FASTDEM_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/third_party/fastdem/config")

  # FastDEM's first example, run as a smoke test. It writes its image into the build tree.
  add_executable(fastdem_example_hello_mapping
    third_party/fastdem/examples/01_hello_mapping/main.cpp)
  target_link_libraries(fastdem_example_hello_mapping fastdem)
  target_compile_options(fastdem_example_hello_mapping PRIVATE -w)
  target_compile_definitions(fastdem_example_hello_mapping PRIVATE
    EXAMPLE_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}")
  add_test(NAME fastdem_example_hello_mapping COMMAND fastdem_example_hello_mapping)
endif()

ament_package()
