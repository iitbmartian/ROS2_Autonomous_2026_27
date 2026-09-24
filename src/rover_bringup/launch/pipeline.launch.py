"""The whole simulated rover stack in one command: Gazebo, EKF, SLAM and Nav2.

    ros2 launch rover_bringup pipeline.launch.py
    ros2 launch rover_bringup pipeline.launch.py world:=marsyard.sdf rviz:=true
    ros2 launch rover_bringup pipeline.launch.py localization:=true

Includes, from this package:

    rover_sim.launch.py         Gazebo, ros2_control, the ros_gz bridge. The only
                                file that starts Gazebo.
    ekf_rover_gazebo.launch.py  wheel encoder chain into robot_localization
    rtabmap.launch.py           /map and map -> odom, mapping or, with
                                localization:=true, against a saved database
    navigation.launch.py        Nav2, with sim:=true

Exactly one node owns each transform in the chain map -> odom -> base_footprint:

    map  -> odom            rtabmap
    odom -> base_footprint  rover_ekf's filter under odom_source:=ekf (the default),
                            otherwise the odometry node rtabmap.launch.py starts
                            for the chosen odom_source

Everything after the simulator is started on a timer. Gazebo needs a moment to
load the world, spawn the model and start the bridge; nodes started before that
log sync and TF warnings until the first message arrives. Nav2 goes last, once
/map and the full TF chain exist for its costmaps.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

# Seconds after launch.
ESTIMATORS_DELAY = 8.0
NAV2_DELAY = 15.0


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory("rover_bringup"), "launch")

    def include(name, arguments):
        # IncludeLaunchDescription does not scope launch arguments, so without
        # the group Nav2's sim:=true would overwrite the pipeline's
        # sim:=harmonic, and so on for every name two files share.
        return GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
                launch_arguments=arguments.items())])

    odom_source = LaunchConfiguration("odom_source")

    sim = include("rover_sim.launch.py", {
        "world": LaunchConfiguration("world"),
        "sim": LaunchConfiguration("sim"),
        "gui": LaunchConfiguration("gui"),
        "teleop": LaunchConfiguration("teleop"),
        "x": LaunchConfiguration("x"),
        "y": LaunchConfiguration("y"),
        "z": LaunchConfiguration("z"),
        "yaw": LaunchConfiguration("yaw"),
    })

    ekf = include("ekf_rover_gazebo.launch.py", {
        "use_imu": LaunchConfiguration("use_imu"),
        "publish_tf": PythonExpression(["'true' if '", odom_source, "' == 'ekf' else 'false'"]),
    })

    slam = include("rtabmap.launch.py", {
        "odom_source": odom_source,
        "localization": LaunchConfiguration("localization"),
        "database_path": LaunchConfiguration("database_path"),
        "delete_db_on_start": LaunchConfiguration("delete_db_on_start"),
        "viz": LaunchConfiguration("viz"),
        "rviz": LaunchConfiguration("rviz"),
        "dense_map": LaunchConfiguration("dense_map"),
    })

    nav2 = include("navigation.launch.py", {"sim": "true"})

    return LaunchDescription([
        # rover_sim.launch.py
        DeclareLaunchArgument("world", default_value="empty_world.sdf",
                              description="empty_world.sdf, flat.sdf, ledge.sdf, bars.sdf, "
                                          "husarion_world.sdf or marsyard.sdf"),
        DeclareLaunchArgument("sim", default_value="harmonic",
                              choices=["harmonic", "fortress"]),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("teleop", default_value="false",
                              description="launch keyboard teleop in this terminal"),
        # husarion_world.sdf needs x:=-13 to spawn clear of its logo plate.
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.02"),
        DeclareLaunchArgument("yaw", default_value="0.0"),

        # ekf_rover_gazebo.launch.py
        DeclareLaunchArgument("use_imu", default_value="false",
                              choices=["true", "false"],
                              description="also fuse the gyro yaw rate into the EKF"),

        # rtabmap.launch.py
        DeclareLaunchArgument("odom_source", default_value="ekf",
                              choices=["ekf", "visual", "lidar", "ground_truth"],
                              description="who publishes odom -> base_footprint. "
                                          "'ekf' is rover_ekf's filter; the others "
                                          "are as in rtabmap.launch.py, and switch "
                                          "the EKF's TF off"),
        DeclareLaunchArgument("localization", default_value="false",
                              choices=["true", "false"],
                              description="RTAB-Map localises against database_path "
                                          "instead of mapping"),
        DeclareLaunchArgument("database_path", default_value="~/.ros/rover_slam.db"),
        DeclareLaunchArgument("delete_db_on_start", default_value="true",
                              choices=["true", "false"]),
        DeclareLaunchArgument("viz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("rviz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("dense_map", default_value="false",
                              choices=["true", "false"]),

        sim,
        TimerAction(period=ESTIMATORS_DELAY, actions=[ekf, slam]),
        TimerAction(period=NAV2_DELAY, actions=[nav2]),
    ])
