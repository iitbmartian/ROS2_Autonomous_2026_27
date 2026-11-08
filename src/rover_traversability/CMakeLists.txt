cmake_minimum_required(VERSION 3.8)
project(rover_traversability)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  # Pose evaluation runs tens of thousands of times per map: unoptimised it is about 100 times
  # slower (measured with rover_terrain_mapping's bench_features).
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# The core first, before anything pulls in grid_map_core (see src/core/CMakeLists.txt).
add_subdirectory(src/core)
add_subdirectory(src/msg)
add_subdirectory(src/critic)
add_subdirectory(src/nodes)

find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(layer_dependencies
  rclcpp pluginlib nav2_costmap_2d nav2_util grid_map_core grid_map_ros grid_map_msgs
  tf2 tf2_ros tf2_geometry_msgs geometry_msgs nav_msgs)

add_library(rover_traversability_layer SHARED src/traversability_layer.cpp)
target_include_directories(rover_traversability_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(rover_traversability_layer traversability_core)
ament_target_dependencies(rover_traversability_layer ${layer_dependencies})

# Behaviour tree node for the 3D tree: approach from behind (ApproachGoals).
find_package(behaviortree_cpp REQUIRED)
add_library(rover_approach_goals_bt_node SHARED src/bt/approach_goals.cpp)
target_include_directories(rover_approach_goals_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_definitions(rover_approach_goals_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(rover_approach_goals_bt_node behaviortree_cpp geometry_msgs)
install(TARGETS rover_approach_goals_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)
pluginlib_export_plugin_description_file(nav2_mppi_controller critics_plugins.xml)

install(TARGETS rover_traversability_layer
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES costmap_plugins.xml DESTINATION share/${PROJECT_NAME})
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_approach_goals test/test_approach_goals.cpp)
  target_link_libraries(test_approach_goals rover_approach_goals_bt_node)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_traversability_layer test/test_traversability_layer.cpp
    TIMEOUT 120 SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_traversability_layer rover_traversability_layer)
  ament_target_dependencies(test_traversability_layer ${layer_dependencies})

  ament_add_gtest(test_traversability_layer_v2 test/test_traversability_layer_v2.cpp
    TIMEOUT 120 SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_traversability_layer_v2 rover_traversability_layer)
  ament_target_dependencies(test_traversability_layer_v2 ${layer_dependencies})

  ament_add_pytest_test(test_physical_params_py test/test_physical_params.py)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${layer_dependencies} yaml-cpp ament_index_cpp Eigen3)
ament_package()
