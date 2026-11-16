# HeadingAwarePlanner, the Nav2 global planner plugin. Its own directory, added before
# grid_map_core is found, for the same reason as src/core.

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_library(rover_heading_aware_planner SHARED heading_aware_planner.cpp)
target_include_directories(rover_heading_aware_planner PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(rover_heading_aware_planner traversability_core grid_map_msg)
ament_target_dependencies(rover_heading_aware_planner
  rclcpp rclcpp_lifecycle pluginlib nav2_core nav2_util nav2_costmap_2d nav_msgs geometry_msgs
  grid_map_msgs tf2)

install(TARGETS rover_heading_aware_planner
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
