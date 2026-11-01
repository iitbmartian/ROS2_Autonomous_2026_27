# traversability_core: pure geometry shared by every consumer (SPEC.md). Built in its own
# directory, added before grid_map_core is found, because grid_map_core's cmake extras add Eigen
# plugin definitions to the whole directory and the core must compile without them.

add_library(traversability_core SHARED
  disc_dilation.cpp
  physical_params.cpp
  pose_evaluation.cpp
  trajectory_cost.cpp
  yaw_evaluation.cpp)
target_include_directories(traversability_core PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
find_package(Threads REQUIRED)
target_link_libraries(traversability_core PUBLIC
  yaml-cpp ament_index_cpp::ament_index_cpp Eigen3::Eigen Threads::Threads)

install(TARGETS traversability_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  set(T ${PROJECT_SOURCE_DIR}/test)

  ament_add_gtest(test_disc_dilation ${T}/test_disc_dilation.cpp)
  target_link_libraries(test_disc_dilation traversability_core)

  ament_add_gtest(test_pose_evaluation ${T}/test_pose_evaluation.cpp)
  target_link_libraries(test_pose_evaluation traversability_core)

  ament_add_gtest(test_yaw_evaluation ${T}/test_yaw_evaluation.cpp)
  target_link_libraries(test_yaw_evaluation traversability_core)

  ament_add_gtest(test_trajectory_cost ${T}/test_trajectory_cost.cpp)
  target_link_libraries(test_trajectory_cost traversability_core)

  ament_add_gtest(test_physical_params ${T}/test_physical_params.cpp)
  target_link_libraries(test_physical_params traversability_core)
  target_compile_definitions(test_physical_params PRIVATE
    TEST_FIXTURE_DIR="${T}/fixtures"
    SHIPPED_PHYSICAL_YAML="${PROJECT_SOURCE_DIR}/config/rover_physical.yaml")
endif()
