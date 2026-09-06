"""Gazebo rover plus RTAB-Map, in one command.

    ros2 launch rover_slam slam_sim.launch.py rviz:=true

then drive it:

    ros2 run rover_gazebo teleop_rover.py

This is rover_gazebo's rover_sim.launch.py and this package's rtabmap.launch.py
included together, with the arguments each one needs passed straight through. Run
them separately if you want to restart SLAM without restarting the simulator.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_share = get_package_share_directory("rover_gazebo")
    slam_share = get_package_share_directory("rover_slam")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, "launch", "rover_sim.launch.py")),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "sim": LaunchConfiguration("sim"),
            "gui": LaunchConfiguration("gui"),
            "teleop": LaunchConfiguration("teleop"),
        }.items())

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, "launch", "rtabmap.launch.py")),
        launch_arguments={
            "odom_source": LaunchConfiguration("odom_source"),
            "localization": LaunchConfiguration("localization"),
            "database_path": LaunchConfiguration("database_path"),
            "delete_db_on_start": LaunchConfiguration("delete_db_on_start"),
            "viz": LaunchConfiguration("viz"),
            "rviz": LaunchConfiguration("rviz"),
        }.items())

    return LaunchDescription([
        # rover_gazebo arguments
        DeclareLaunchArgument("world", default_value="flat.sdf",
                              description="flat.sdf, ledge.sdf or bars.sdf"),
        DeclareLaunchArgument("sim", default_value="harmonic",
                              choices=["harmonic", "fortress"]),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("teleop", default_value="false"),

        # rover_slam arguments
        DeclareLaunchArgument("odom_source", default_value="ground_truth",
                              choices=["visual", "lidar", "ground_truth"]),
        DeclareLaunchArgument("localization", default_value="false",
                              choices=["true", "false"]),
        DeclareLaunchArgument("database_path",
                              default_value="~/.ros/rover_slam.db"),
        DeclareLaunchArgument("delete_db_on_start", default_value="true",
                              choices=["true", "false"]),
        DeclareLaunchArgument("viz", default_value="false",
                              choices=["true", "false"]),
        DeclareLaunchArgument("rviz", default_value="false",
                              choices=["true", "false"]),

        sim,
        # Gazebo needs a moment to load the world, spawn the model and start the
        # bridge. Starting RTAB-Map into an empty graph makes it log sync
        # warnings for several seconds before the first frame arrives.
        TimerAction(period=8.0, actions=[slam]),
    ])
