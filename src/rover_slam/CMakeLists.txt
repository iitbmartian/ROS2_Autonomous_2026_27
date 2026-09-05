cmake_minimum_required(VERSION 3.8)
project(rover_slam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz doc
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  rover_slam/gt_odom_tf.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
