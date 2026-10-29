"""rover_bringup's ekf_rover_gazebo.launch.py, unchanged.

    ros2 launch rover_testing rover_sim.launch.py
    ros2 launch rover_testing ekf_rover_gazebo.launch.py

A thin wrapper: it includes the rover_bringup file, so it always runs the current
bringup code. The EKF has no display or world to override, so no default differs;
every argument (use_imu, noisy_wheels, use_gps, publish_tf, ...) passes straight
through from the command line.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup = os.path.join(get_package_share_directory("rover_bringup"), "launch",
                           "ekf_rover_gazebo.launch.py")
    return LaunchDescription([GroupAction(scoped=True, actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup))])])
