cmake_minimum_required(VERSION 3.8)
project(rover_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch config worlds rviz doc tools
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  rover_gazebo/rover_kinematics_node.py
  rover_gazebo/rocker_coupling_node.py
  rover_gazebo/rocker_hold_node.py
  rover_gazebo/control_node.py
  rover_gazebo/teleop_rover.py
  rover_gazebo/teleop_rover_gui.py
  rover_gazebo/measure_rover.py
  tools/derive_ratios.py
  tools/generate_description.py
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
