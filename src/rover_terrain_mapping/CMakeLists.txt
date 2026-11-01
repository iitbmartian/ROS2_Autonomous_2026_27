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

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(rover_traversability REQUIRED)
# rover_traversability exports grid_map_core, whose cmake extras add Eigen plugin definitions to
# the whole directory. This package uses no grid_map_core, and the plugins must not reach
# FastDEM's or nanoGrid's Eigen code.
remove_definitions(
  -DEIGEN_FUNCTORS_PLUGIN=\"grid_map_core/eigen_plugins/FunctorsPlugin.hpp\"
  -DEIGEN_DENSEBASE_PLUGIN=\"grid_map_core/eigen_plugins/DenseBasePlugin.hpp\")

add_library(contract_layers STATIC src/contract_layers.cpp)
target_include_directories(contract_layers PUBLIC include)
target_link_libraries(contract_layers PUBLIC fastdem)
# Only layer_names.hpp is used. Linking traversability_core, not the package, keeps
# grid_map_core's Eigen plugin definitions out of FastDEM's translation units.
target_link_libraries(contract_layers PUBLIC rover_traversability::traversability_core)

# Feature math (SPEC.md layer contract), plain C++ on Eigen.
find_package(Eigen3 REQUIRED NO_MODULE)
add_library(terrain_features STATIC src/features.cpp)
target_include_directories(terrain_features PUBLIC include)
find_package(Threads REQUIRED)
target_link_libraries(terrain_features PUBLIC Eigen3::Eigen Threads::Threads)
target_compile_options(terrain_features PRIVATE -Wall -Wextra -Wpedantic -O2)

add_library(tf_provider STATIC src/tf_provider.cpp)
target_include_directories(tf_provider PUBLIC include)
target_link_libraries(tf_provider PUBLIC fastdem)
ament_target_dependencies(tf_provider PUBLIC rclcpp tf2 tf2_ros tf2_eigen)

add_executable(terrain_mapping_node src/terrain_mapping_node.cpp)
target_include_directories(terrain_mapping_node PRIVATE include)
target_link_libraries(terrain_mapping_node contract_layers tf_provider fastdem
  rover_traversability::traversability_core)
ament_target_dependencies(terrain_mapping_node
  rclcpp sensor_msgs grid_map_msgs std_msgs visualization_msgs tf2 tf2_ros tf2_eigen)

add_library(grid_map_msg STATIC src/grid_map_msg.cpp)
target_include_directories(grid_map_msg PUBLIC include)
target_link_libraries(grid_map_msg PUBLIC Eigen3::Eigen)
ament_target_dependencies(grid_map_msg PUBLIC grid_map_msgs std_msgs)

add_executable(terrain_features_node src/terrain_features_node.cpp)
target_link_libraries(terrain_features_node
  terrain_features grid_map_msg rover_traversability::traversability_core)
ament_target_dependencies(terrain_features_node rclcpp grid_map_msgs tf2 tf2_ros)

add_library(depth_range_cut STATIC src/depth_range_cut.cpp)
target_include_directories(depth_range_cut PUBLIC include)
target_link_libraries(depth_range_cut PUBLIC fastdem)

add_executable(depth_range_cut_node src/depth_range_cut_node.cpp)
target_link_libraries(depth_range_cut_node depth_range_cut)
ament_target_dependencies(depth_range_cut_node rclcpp sensor_msgs)

# CPU-time benchmark of one feature map (not a test, not installed).
add_executable(bench_features tools/bench_features.cpp)
target_link_libraries(bench_features terrain_features rover_traversability::traversability_core)

install(TARGETS terrain_mapping_node terrain_features_node depth_range_cut_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

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

  ament_add_gtest(test_contract_layers test/test_contract_layers.cpp)
  target_link_libraries(test_contract_layers contract_layers)

  ament_add_gtest(test_features test/test_features.cpp)
  target_link_libraries(test_features terrain_features)

  ament_add_gtest(test_depth_range_cut test/test_depth_range_cut.cpp)
  target_link_libraries(test_depth_range_cut depth_range_cut)

  ament_add_gtest(test_tf_provider test/test_tf_provider.cpp)
  target_link_libraries(test_tf_provider tf_provider)

  ament_add_gtest(test_grid_map_msg test/test_grid_map_msg.cpp)
  target_link_libraries(test_grid_map_msg grid_map_msg)
endif()

ament_package()
