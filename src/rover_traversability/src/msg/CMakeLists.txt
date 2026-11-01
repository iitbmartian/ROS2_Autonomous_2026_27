# grid_map_msgs/GridMap to and from Eigen matrices, without grid_map_core (whose Eigen plugin
# definitions must not reach FastDEM or traversability_core). Added before grid_map_core is found.

find_package(grid_map_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(grid_map_msg SHARED grid_map_msg.cpp)
target_include_directories(grid_map_msg PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(grid_map_msg PUBLIC Eigen3::Eigen)
ament_target_dependencies(grid_map_msg PUBLIC grid_map_msgs std_msgs)

install(TARGETS grid_map_msg
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_grid_map_msg ${PROJECT_SOURCE_DIR}/test/test_grid_map_msg.cpp)
  target_link_libraries(test_grid_map_msg grid_map_msg)
endif()
