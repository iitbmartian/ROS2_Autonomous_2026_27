"""Terrain mapping for the 3D navigation stack: terrain_mapping_node on FastDEM.

    ros2 launch rover_bringup terrain_mapping.launch.py              # simulation
    ros2 launch rover_bringup terrain_mapping.launch.py sim:=false
    ros2 launch rover_bringup terrain_mapping.launch.py features:=fastdem   # Phase 0 bridge

Publishes /terrain/local (elevation, variance, observations) and /terrain/local_features
(slope, slope_std, step, roughness, curvature, normals, negative_obstacle), both
grid_map_msgs/GridMap in odom. features:=native (default) computes the feature layers in
terrain_features_node with the definitions of SPEC.md. features:=fastdem publishes FastDEM's
own features instead, the Phase 0 bridge, kept for comparison.
Needs the lidar on /lidar/points and the TF chain odom -> base_footprint -> lidar.
See rover_nav2/doc/3d_nav/SPEC.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('rover_terrain_mapping')
    config = os.path.join(share, 'config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='true',
            description='Use /clock. Only the simulation configuration exists so far'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(config, 'terrain_mapping_sim.yaml'),
            description='terrain_mapping_node ROS parameters'),
        DeclareLaunchArgument(
            'features', default_value='native', choices=['native', 'fastdem'],
            description='Feature layers from terrain_features_node or from FastDEM'),
        DeclareLaunchArgument(
            'sensor_frame', default_value='lidar3d_link',
            description='Frame of the lidar, for negative-obstacle occlusion reasoning'),
        DeclareLaunchArgument(
            'fastdem_config', default_value=os.path.join(config, 'fastdem_sim.yaml'),
            description='FastDEM pipeline file, in FastDEM format'),
        Node(
            package='rover_terrain_mapping', executable='terrain_mapping_node',
            name='terrain_mapping', output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'fastdem_config': LaunchConfiguration('fastdem_config'),
                 'use_sim_time': LaunchConfiguration('sim'),
                 'publish_features': PythonExpression(
                     ["'", LaunchConfiguration('features'), "' == 'fastdem'"])},
            ]),
        Node(
            package='rover_terrain_mapping', executable='terrain_features_node',
            name='terrain_features', output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('sim'),
                         'sensor_frame': LaunchConfiguration('sensor_frame')}],
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('features'), "' == 'native'"]))),
    ])
