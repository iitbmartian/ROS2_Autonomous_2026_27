"""The whole simulated rover stack, Gazebo, EKF, RTAB-Map and Nav2, started for testing.

    ros2 launch rover_testing pipeline.launch.py
    ros2 launch rover_testing pipeline.launch.py gui:=true nav_rviz:=true
    ros2 launch rover_testing pipeline.launch.py world:=/abs/path/to/slopes.sdf
    ros2 launch rover_testing pipeline.launch.py nav2:=false        # drive tests

This is what every test case in launch/tests/ starts. It runs the same four
rover_bringup component files rover_bringup's pipeline.launch.py does, with the
same arguments, so the code under test is always the current bringup code:

    rover_sim.launch.py, ekf_rover_gazebo.launch.py, rtabmap.launch.py,
    navigation.launch.py (sim:=true)

plus the same depth-image obstacle cloud and optional Nav2 RViz. It is not a plain
include of bringup's pipeline.launch.py because of two things that file hard-codes:

Startup timers. bringup waits 8 s before the EKF and RTAB-Map and 15 s before
Nav2. Here everything starts at once (estimators_delay and nav2_delay default to
0). Nothing needs the wait to be correct: the EKF, navsat_transform and RTAB-Map
sit idle until /clock and their sensors arrive, and Nav2 configures without TF,
then its costmaps hold activation until map -> odom -> base_footprint exists.
Nav2's configure is the long pole, about 15 s of Smac's Reeds-Shepp lookup-table
precompute (lookup_table_size: 20 m, 72 angle bins), and starting it at t=0
overlaps that with Gazebo's own startup instead of queueing behind it. Measured
on empty_world.sdf, the first goal went out at 38.6 s with bringup's timers. See
README.md for the time with them gone.

Isolation. Each launch gets its own GZ_PARTITION and a shutdown hook that kills
any Gazebo server the launch orphans (rover_testing/gz_reaper.py). Without them a
leftover server from one run silently hijacks the next one's bridge.

Test-friendly defaults, otherwise as bringup:

    world          empty_world.sdf (a bare name is looked up in rover_gazebo/worlds;
                   an absolute path is used as is)
    gui            false  (gz server only, headless rendering)
    teleop_gui     false
    x, y, yaw      0, 0, 0
    viz, rviz      false
    nav_rviz       false
    database_path  ~/.ros/rover_testing/rover_slam_test.db, so a test run never
                   wipes the database rover_bringup maps into
    nav2           true; false leaves Nav2 out entirely, for tests that drive
                   /cmd_vel themselves
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from rover_testing.gz_reaper import isolate_and_reap


def generate_launch_description():
    bringup = os.path.join(get_package_share_directory("rover_bringup"), "launch")
    os.makedirs(os.path.expanduser("~/.ros/rover_testing"), exist_ok=True)

    def include(name, arguments, **kwargs):
        # Scoped, as in bringup's pipeline: otherwise Nav2's sim:=true would
        # overwrite the pipeline's sim:=harmonic, and so on for every shared name.
        return GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup, name)),
                launch_arguments=arguments.items())], **kwargs)

    def arg(name):
        return LaunchConfiguration(name)

    odom_source = arg("odom_source")

    sim = include("rover_sim.launch.py", {
        name: arg(name) for name in (
            "world", "sim", "gui", "teleop", "teleop_gui", "suspension", "physics_engine",
            "x", "y", "z", "yaw")})

    ekf = include("ekf_rover_gazebo.launch.py", {
        "use_imu": arg("use_imu"),
        "noisy_wheels": arg("noisy_wheels"),
        "publish_tf": PythonExpression(["'true' if '", odom_source, "' == 'ekf' else 'false'"]),
    })

    slam = include("rtabmap.launch.py", {
        "odom_source": odom_source,
        **{name: arg(name) for name in (
            "localization", "database_path", "delete_db_on_start", "viz", "rviz",
            "dense_map")},
    })

    nav2 = include("navigation.launch.py", {"sim": "true"})

    # Copied from rover_bringup's pipeline.launch.py, which defines it inline; see
    # the comment there for why the cloud is rebuilt from the depth image.
    depth_cloud = Node(
        package="rtabmap_util", executable="point_cloud_xyz", name="depth_cloud",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "approx_sync": True,
            "qos": 1,
            "qos_camera_info": 1,
            "decimation": 4,
            "voxel_size": 0.05,
            "min_depth": 0.3,
            "max_depth": 4.5,
        }],
        remappings=[
            ("depth/image", "/camera/depth_image"),
            ("depth/camera_info", "/camera/camera_info"),
            ("cloud", "/camera/depth_points"),
        ])

    nav_rviz = Node(
        package="rviz2", executable="rviz2", output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["-d", os.path.join(get_package_share_directory("rover_nav2"),
                                      "rviz", "nav2_sim.rviz")],
        condition=IfCondition(arg("nav_rviz")))

    # The delays are read here, while this file's arguments are in scope. A
    # TimerAction given LaunchConfiguration("...") as its period evaluates it in
    # an asyncio task after an including scoped GroupAction has popped them, fails
    # with "launch configuration ... does not exist", and never fires; the same
    # trap rover_bringup's rover_sim.launch.py documents for its event handlers.
    def staged(context):
        def delayed(name, actions):
            period = float(arg(name).perform(context))
            return [TimerAction(period=period, actions=actions)] if period > 0 else actions
        actions = delayed("estimators_delay", [ekf, slam, depth_cloud])
        if arg("nav2").perform(context).lower() in ("true", "1"):
            actions += delayed("nav2_delay", [nav2])
        return actions

    def declare(name, default, description=""):
        return DeclareLaunchArgument(name, default_value=default, description=description)

    return LaunchDescription([
        # rover_sim.launch.py
        declare("world", "empty_world.sdf",
                "file in rover_gazebo/worlds, or an absolute path to a .sdf"),
        declare("sim", "harmonic"),
        declare("gui", "false", "show the Gazebo GUI"),
        declare("teleop", "false"),
        declare("teleop_gui", "false"),
        declare("suspension", "torque"),
        declare("physics_engine", ""),
        declare("x", "0.0"),
        declare("y", "0.0"),
        declare("z", "0.02"),
        declare("yaw", "0.0"),
        # ekf_rover_gazebo.launch.py
        declare("use_imu", "true"),
        declare("noisy_wheels", "false"),
        # rtabmap.launch.py
        declare("odom_source", "ekf"),
        declare("localization", "false"),
        declare("database_path", "~/.ros/rover_testing/rover_slam_test.db"),
        declare("delete_db_on_start", "true"),
        declare("viz", "false"),
        declare("rviz", "false"),
        declare("dense_map", "false"),
        # Nav2
        declare("nav2", "true", "start Nav2; false for tests that drive /cmd_vel directly"),
        declare("nav_rviz", "false"),
        # Startup staging, in seconds after launch. bringup's pipeline uses 8 and 15.
        declare("estimators_delay", "0.0", "start the EKF, RTAB-Map and depth cloud after"),
        declare("nav2_delay", "0.0", "start Nav2 after"),

        *isolate_and_reap(),
        sim,
        nav_rviz,
        OpaqueFunction(function=staged),
    ])
