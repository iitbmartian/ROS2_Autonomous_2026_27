"""The simulated rover stack with 3D terrain navigation.

    ros2 launch rover_bringup pipeline_3d.launch.py world:=marsyard.sdf
    ros2 launch rover_bringup pipeline_3d.launch.py world:=marsyard.sdf gui:=false terrain_rviz:=true

Runs pipeline.launch.py (Gazebo, EKF, RTAB-Map, Nav2) with the 3D Nav2 parameters, plus
terrain_mapping.launch.py. The 3D parameters are rover_nav2's nav2_params_sim_3d.yaml merged
over nav2_params_sim.yaml at launch time (rover_nav2.params_overlay), written to a file under
/tmp and passed to the pipeline as nav_params_file. The 2D stack and its files are untouched:
pipeline.launch.py without nav_params_file still runs the 2D tuning.

Every pipeline.launch.py argument listed below is forwarded unchanged.

The lidar defaults to the L2-like dome, mounted inverted (rover_gazebo's lidar_model, read from
ROVER_LIDAR_MODEL, ROVER_LIDAR_INVERTED, ROVER_LIDAR_TILT, which this file sets). lidar_model:=default
keeps the 16-ring roof lidar. The features nodes get the matching lowest beam angle for
negative-obstacle reasoning.
"""

import math
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rover_nav2.params_overlay import merge_files

# Seconds after launch, same as pipeline.launch.py's estimators: the mapper needs the TF chain.
TERRAIN_DELAY = 8.0

# pipeline.launch.py arguments forwarded as they are, with the same defaults.
FORWARDED = {
    'world': 'empty_world.sdf',
    'gui': 'true',
    'teleop_gui': 'false',
    'x': '0.0',
    'y': '0.0',
    'z': '0.02',
    'yaw': '0.0',
    'odom_source': 'ekf',
    'delete_db_on_start': 'true',
    'database_path': '~/.ros/rover_slam.db',
    'viz': 'true',
    'rviz': 'false',
    'nav_rviz': 'false',
}


def _launch(context):
    launch_dir = os.path.join(get_package_share_directory('rover_bringup'), 'launch')
    nav2_config = os.path.join(get_package_share_directory('rover_nav2'), 'config')

    merged = os.path.join(
        tempfile.gettempdir(), f'rover_nav2_params_sim_3d_{os.getpid()}.yaml')
    merge_files(
        os.path.join(nav2_config, 'nav2_params_sim.yaml'),
        LaunchConfiguration('nav_overlay').perform(context),
        merged)

    args = {k: LaunchConfiguration(k) for k in FORWARDED}
    args['nav_params_file'] = merged

    model = LaunchConfiguration('lidar_model').perform(context)
    inverted = LaunchConfiguration('lidar_inverted').perform(context).lower() in ('true', '1')
    tilt = float(LaunchConfiguration('lidar_tilt').perform(context))
    # Lowest beam in the world frame: the roof lidar's -15 deg, or the L2 dome's -6 deg turned
    # upside down (-90 deg reaches straight down), lowered further by a nose-down tilt.
    if model == 'l2':
        lowest = -math.pi / 2 if inverted else -0.1047 - tilt
    else:
        lowest = -0.2618
    env = [SetEnvironmentVariable('ROVER_LIDAR_MODEL', model),
           SetEnvironmentVariable('ROVER_LIDAR_INVERTED', 'true' if inverted else 'false'),
           SetEnvironmentVariable('ROVER_LIDAR_TILT', str(tilt))]

    pipeline = GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'pipeline.launch.py')),
        launch_arguments=args.items())])
    terrain = GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'terrain_mapping.launch.py')),
        launch_arguments={'sim': 'true', 'lowest_beam': str(lowest)}.items())])
    # nav2_sim.rviz plus the terrain elevation and slope maps (grid_map_rviz_plugin).
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', os.path.join(get_package_share_directory('rover_nav2'),
                                      'rviz', 'terrain_3d.rviz')],
        condition=IfCondition(LaunchConfiguration('terrain_rviz')))
    # Speed governor and safety monitor (SPEC.md speed and safety), each switchable for the
    # phase comparisons in the benchmark.
    safety_nodes = []
    if LaunchConfiguration('governor').perform(context).lower() in ('true', '1'):
        safety_nodes.append(Node(
            package='rover_traversability', executable='terrain_speed_governor',
            output='screen', parameters=[{'use_sim_time': True}]))
    if LaunchConfiguration('safety').perform(context).lower() in ('true', '1'):
        safety_nodes.append(Node(
            package='rover_traversability', executable='terrain_safety_monitor',
            output='screen', parameters=[{'use_sim_time': True}]))
    return env + [pipeline, TimerAction(period=TERRAIN_DELAY, actions=[terrain] + safety_nodes),
                  rviz]


def generate_launch_description():
    overlay = os.path.join(
        get_package_share_directory('rover_nav2'), 'config', 'nav2_params_sim_3d.yaml')
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in FORWARDED.items()]
        + [DeclareLaunchArgument('nav_overlay', default_value=overlay,
                                 description='Overlay merged over nav2_params_sim.yaml'),
           DeclareLaunchArgument('lidar_model', default_value='l2', choices=['l2', 'default'],
                                 description='l2: L2-like dome lidar. default: 16-ring roof lidar'),
           DeclareLaunchArgument('lidar_inverted', default_value='true',
                                 description='Mount the L2 upside down so the dome sees the ground'),
           DeclareLaunchArgument('lidar_tilt', default_value='0.0',
                                 description='L2 nose-down mount tilt, rad'),
           DeclareLaunchArgument('governor', default_value='true',
                                 description='terrain_speed_governor on /speed_limit'),
           DeclareLaunchArgument('safety', default_value='true',
                                 description='terrain_safety_monitor: hazard cloud, tip abort'),
           DeclareLaunchArgument('terrain_rviz', default_value='false',
                                 description='RViz with the Nav2 view and the terrain maps'),
           OpaqueFunction(function=_launch)])
