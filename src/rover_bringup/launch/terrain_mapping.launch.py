"""Terrain mapping for the 3D navigation stack: terrain_mapping_node on FastDEM.

    ros2 launch rover_bringup terrain_mapping.launch.py              # simulation
    ros2 launch rover_bringup terrain_mapping.launch.py sim:=false depth_range_cut:=false
    ros2 launch rover_bringup terrain_mapping.launch.py features:=fastdem   # Phase 0 bridge

Starts depth_range_cut (the Orin stand-in: /camera/depth_points cropped and downsampled to
/terrain/depth_points), terrain_mapping_node (lidar and depth into FastDEM) and, by default,
terrain_features_node.

Publishes /terrain/local (elevation, variance, observations, last_seen), /terrain/global
(coarse, in map) and /terrain/local_features
(slope, slope_std, step, roughness, curvature, normals, negative_obstacle), both
grid_map_msgs/GridMap in odom. features:=native (default) computes the feature layers in
terrain_features_node with the definitions of SPEC.md. features:=fastdem publishes FastDEM's
own features instead, the Phase 0 bridge, kept for comparison.
Needs the lidar on /lidar/points and the TF chain odom -> base_footprint -> lidar.
sim:=false switches the default parameter files to terrain_mapping_real.yaml and fastdem_real.yaml
(the L2 on /unilidar/cloud, gravity from the IMU). On the rover depth_range_cut runs on the Orin,
so depth_range_cut:=false leaves it out here. base_frame overrides the mapping node's body frame.
See rover_nav2/doc/3d_nav/SPEC.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _mapping_node(context, config):
    sim = LaunchConfiguration('sim').perform(context).lower() in ('true', '1')
    kind = 'sim' if sim else 'real'
    params = LaunchConfiguration('params_file').perform(context) or os.path.join(
        config, f'terrain_mapping_{kind}.yaml')
    fastdem = LaunchConfiguration('fastdem_config').perform(context) or os.path.join(
        config, f'fastdem_{kind}.yaml')
    over = {'fastdem_config': fastdem, 'use_sim_time': sim,
            'publish_features': LaunchConfiguration('features').perform(context) == 'fastdem'}
    base = LaunchConfiguration('base_frame').perform(context)
    if base:
        over['base_frame'] = base
    return [Node(
        package='rover_terrain_mapping', executable='terrain_mapping_node',
        name='terrain_mapping', output='screen', parameters=[params, over])]


def generate_launch_description():
    share = get_package_share_directory('rover_terrain_mapping')
    config = os.path.join(share, 'config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='true',
            description='Use /clock and the simulation parameter files'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='terrain_mapping_node ROS parameters, empty for terrain_mapping_sim.yaml '
                        'or terrain_mapping_real.yaml by sim'),
        DeclareLaunchArgument(
            'depth_range_cut', default_value='true',
            description='Run depth_range_cut here. false on the rover, where the Orin runs it'),
        DeclareLaunchArgument(
            'depth_input_topic', default_value='/camera/depth_points',
            description='depth_range_cut input cloud'),
        DeclareLaunchArgument(
            'base_frame', default_value='',
            description='Mapping node body frame, empty keeps the parameter file\'s'),
        DeclareLaunchArgument(
            'features', default_value='native', choices=['native', 'fastdem'],
            description='Feature layers from terrain_features_node or from FastDEM'),
        DeclareLaunchArgument(
            'sensor_frame', default_value='lidar3d_link',
            description='Frame of the lidar, for negative-obstacle occlusion reasoning'),
        DeclareLaunchArgument(
            'lowest_beam', default_value='-0.2618',
            description='Lidar lowest beam angle (rad, negative is down) for negative-obstacle '
                        'reasoning: -0.2618 for the 16-ring roof lidar, -1.5708 for an inverted L2'),
        DeclareLaunchArgument(
            'fill_holes', default_value='false',
            description='Fill small enclosed unknown patches (below negative.min_gap) before '
                        'computing features, Laplace interpolation, extra variance'),
        DeclareLaunchArgument(
            'fastdem_config', default_value='',
            description='FastDEM pipeline file, in FastDEM format, empty for fastdem_sim.yaml or '
                        'fastdem_real.yaml by sim'),
        Node(
            package='rover_terrain_mapping', executable='depth_range_cut_node',
            name='depth_range_cut', output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('sim'),
                         'input_topic': LaunchConfiguration('depth_input_topic'),
                         'max_range': 4.0, 'voxel_size': 0.05}],
            condition=IfCondition(LaunchConfiguration('depth_range_cut'))),
        OpaqueFunction(function=_mapping_node, kwargs={'config': config}),
        Node(
            package='rover_terrain_mapping', executable='terrain_features_node',
            name='terrain_features', output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('sim'),
                         'sensor_frame': LaunchConfiguration('sensor_frame'),
                         'negative.min_beam_angle': LaunchConfiguration('lowest_beam'),
                         'fill_holes': LaunchConfiguration('fill_holes')}],
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('features'), "' == 'native'"]))),
        # Second instance on the coarse global map, for the global costmap.
        Node(
            package='rover_terrain_mapping', executable='terrain_features_node',
            name='terrain_features_global', output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('sim'),
                         'input_topic': '/terrain/global',
                         'output_topic': '/terrain/global_features',
                         'sensor_frame': LaunchConfiguration('sensor_frame'),
                         'negative.min_beam_angle': LaunchConfiguration('lowest_beam'),
                         'fill_holes': LaunchConfiguration('fill_holes')}],
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('features'), "' == 'native'"]))),
    ])
