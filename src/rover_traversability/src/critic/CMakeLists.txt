# TerrainCritic, the MPPI critic plugin. Its own directory, added before grid_map_core is found,
# for the same reason as src/core.

find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(grid_map_msgs REQUIRED)

add_library(rover_terrain_critic SHARED terrain_critic.cpp)
target_include_directories(rover_terrain_critic PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(rover_terrain_critic traversability_core grid_map_msg)
ament_target_dependencies(rover_terrain_critic
  rclcpp pluginlib nav2_mppi_controller nav2_costmap_2d grid_map_msgs)

# The plugin description is exported from the top-level CMakeLists.txt, since pluginlib resolves
# it relative to the calling directory.

install(TARGETS rover_terrain_critic
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_terrain_critic_plugin ${PROJECT_SOURCE_DIR}/test/test_terrain_critic_plugin.cpp)
  ament_target_dependencies(test_terrain_critic_plugin pluginlib nav2_mppi_controller)
endif()
