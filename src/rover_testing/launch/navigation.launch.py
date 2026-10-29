"""rover_bringup's navigation.launch.py, defaulting to sim:=true.

    ros2 launch rover_testing navigation.launch.py

A thin wrapper: it includes the rover_bringup file, so it always runs the current
bringup code. Every test here runs in Gazebo, so sim defaults to true (the
simulation Nav2 tuning); every other argument passes straight through.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

OVERRIDES = {
    "sim": "true",
}


def generate_launch_description():
    bringup = os.path.join(get_package_share_directory("rover_bringup"), "launch",
                           "navigation.launch.py")
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value) for name, value in OVERRIDES.items()]
        + [GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments={name: LaunchConfiguration(name) for name in OVERRIDES}.items())])])
