# terrain_speed_governor and terrain_safety_monitor. Own directory, added before grid_map_core is
# found, like src/core.

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

foreach(n terrain_speed_governor terrain_safety_monitor)
  add_executable(${n} ${n}.cpp)
  target_link_libraries(${n} traversability_core grid_map_msg)
  ament_target_dependencies(${n}
    rclcpp rclcpp_action nav2_msgs nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs)
  install(TARGETS ${n} DESTINATION lib/${PROJECT_NAME})
endforeach()
