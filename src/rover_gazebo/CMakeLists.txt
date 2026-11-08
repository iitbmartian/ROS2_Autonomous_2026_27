cmake_minimum_required(VERSION 3.8)
project(rover_gazebo)

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)

# suspension:=plugin: the rocker coupling inside the physics step.
add_library(rover_gazebo_rocker_coupling SHARED src/rocker_coupling_system.cpp)
target_compile_features(rover_gazebo_rocker_coupling PRIVATE cxx_std_17)
target_link_libraries(rover_gazebo_rocker_coupling gz-sim::gz-sim gz-plugin::register)
install(TARGETS rover_gazebo_rocker_coupling LIBRARY DESTINATION lib)

install(DIRECTORY urdf meshes models config worlds rviz doc tools
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  rover_gazebo/rover_kinematics_node.py
  rover_gazebo/rocker_coupling_node.py
  rover_gazebo/rocker_hold_node.py
  rover_gazebo/control_node.py
  rover_gazebo/teleop_rover.py
  rover_gazebo/teleop_rover_gui.py
  rover_gazebo/measure_rover.py
  rover_gazebo/stability_test.py
  tools/derive_ratios.py
  tools/generate_description.py
  tools/run_stability_suite.sh
  DESTINATION lib/${PROJECT_NAME})

# Shared driving logic imported by teleop_rover.py and teleop_rover_gui.py -- a plain
# file, not executable, installed alongside them so it's importable via sys.path[0]
# (this package installs scripts directly rather than as an ament_python package).
install(FILES
  rover_gazebo/teleop_common.py
  DESTINATION lib/${PROJECT_NAME})

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/resource_paths.dsv.in")

ament_package()
