"""The whole simulated rover stack in one command: Gazebo, EKF, SLAM and Nav2.

    ros2 launch rover_bringup pipeline.launch.py
    ros2 launch rover_bringup pipeline.launch.py world:=marsyard.sdf rviz:=true
    ros2 launch rover_bringup pipeline.launch.py localization:=true
    ros2 launch rover_bringup pipeline.launch.py nav_rviz:=true   # RViz with the Nav2 Goal tool
    ros2 launch rover_bringup pipeline.launch.py teleop_gui:=true # Tk keyboard teleop window
    ros2 launch rover_bringup pipeline.launch.py suspension:=mimic \
        physics_engine:=gz-physics-bullet-featherstone-plugin

Includes, from this package:

    rover_sim.launch.py         Gazebo, ros2_control, the ros_gz bridge. The only
                                file that starts Gazebo.
    ekf_rover_gazebo.launch.py  wheel encoder chain into robot_localization
    rtabmap.launch.py           /map and map -> odom, mapping or, with
                                localization:=true, against a saved database
    navigation.launch.py        Nav2, with sim:=true

plus a depth-image obstacle cloud on /camera/depth_points for Nav2's local
costmap, and, with nav_rviz:=true, RViz on rover_nav2's nav2_sim.rviz.

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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

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
        "teleop_gui": LaunchConfiguration("teleop_gui"),
        "suspension": LaunchConfiguration("suspension"),
        "physics_engine": LaunchConfiguration("physics_engine"),
        "x": LaunchConfiguration("x"),
        "y": LaunchConfiguration("y"),
        "z": LaunchConfiguration("z"),
        "yaw": LaunchConfiguration("yaw"),
    })

    ekf = include("ekf_rover_gazebo.launch.py", {
        "use_imu": LaunchConfiguration("use_imu"),
        "noisy_wheels": LaunchConfiguration("noisy_wheels"),
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

    # Near-field obstacle cloud for the local costmap, rebuilt from the depth
    # image. rover_gazebo's own /camera/points cannot be used: Gazebo publishes
    # that cloud in the camera's body axes (x forward, z up) but it is stamped
    # camera_optical_frame (z forward), which rover_gazebo needs for the images.
    # Read through TF, the ground ahead turns into a phantom wall beside the
    # rover. The depth image carries the optical convention its frame promises,
    # so a cloud projected from it lands where it should.
    depth_cloud = Node(
        package="rtabmap_util", executable="point_cloud_xyz", name="depth_cloud",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "approx_sync": True,
            "qos": 1,                 # ros_gz_bridge publishes reliable
            "qos_camera_info": 1,
            "decimation": 4,          # 640x360 -> 160x90, plenty for a 5 cm costmap
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
        condition=IfCondition(LaunchConfiguration("nav_rviz")))

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
        DeclareLaunchArgument("teleop_gui", default_value="false",
                              description="launch the Tkinter keyboard teleop window"),
        # The suspension's closed loops: 'torque' (rocker_coupling_node, any engine)
        # or 'mimic', which needs
        # physics_engine:=gz-physics-bullet-featherstone-plugin alongside it.
        DeclareLaunchArgument("suspension", default_value="torque",
                              choices=["torque", "mimic"]),
        DeclareLaunchArgument("physics_engine", default_value="",
                              description="gz sim --physics-engine; empty keeps DART"),
        # husarion_world.sdf needs x:=-13 to spawn clear of its logo plate.
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.02"),
        DeclareLaunchArgument("yaw", default_value="0.0"),

        # ekf_rover_gazebo.launch.py
        # On by default: the IMU supplies the roll and pitch RTAB-Map needs to
        # keep tilted lidar rings off the occupancy grid, and a far steadier yaw
        # rate than the wheels.
        DeclareLaunchArgument("use_imu", default_value="true",
                              choices=["true", "false"],
                              description="also fuse the IMU (roll, pitch, angular "
                                          "rates) into the EKF"),
        DeclareLaunchArgument("noisy_wheels", default_value="false",
                              choices=["true", "false"],
                              description="inject the EKF test noise into the wheel "
                                          "encoders (rover_ekf steer_wheels_noisy.yaml)"),

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

        # Nav2
        DeclareLaunchArgument("nav_rviz", default_value="false", choices=["true", "false"],
                              description="RViz with the costmaps, plans and Nav2 Goal tool"),

        sim,
        TimerAction(period=ESTIMATORS_DELAY, actions=[ekf, slam]),
        TimerAction(period=ESTIMATORS_DELAY, actions=[depth_cloud]),
        nav_rviz,
        TimerAction(period=NAV2_DELAY, actions=[nav2]),
    ])
