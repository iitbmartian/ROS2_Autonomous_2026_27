"""rover_bringup's rtabmap.launch.py with test-friendly defaults.

    ros2 launch rover_testing rover_sim.launch.py
    ros2 launch rover_testing rtabmap.launch.py

A thin wrapper: it includes the rover_bringup file, so it always runs the current
bringup code. Only the defaults below differ; every other rtabmap argument passes
straight through from the command line.

    viz            false  (no rtabmap_viz window)
    rviz           false
    database_path  ~/.ros/rover_testing/rover_slam_test.db, so a test run, which
                   deletes its database on start, never wipes the one
                   rover_bringup maps into
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

OVERRIDES = {
    "viz": "false",
    "rviz": "false",
    "database_path": "~/.ros/rover_testing/rover_slam_test.db",
}


def generate_launch_description():
    bringup = os.path.join(get_package_share_directory("rover_bringup"), "launch",
                           "rtabmap.launch.py")
    os.makedirs(os.path.expanduser("~/.ros/rover_testing"), exist_ok=True)
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value) for name, value in OVERRIDES.items()]
        + [GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments={name: LaunchConfiguration(name) for name in OVERRIDES}.items())])])
