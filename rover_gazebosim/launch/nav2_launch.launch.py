


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg = get_package_share_directory('rover_gazebosim')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    params_file = os.path.join(pkg, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg,'maps', 'my_map_final.yaml')
    

    return LaunchDescription([

        # 1. map_server — loads saved map, publishes /map
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'use_sim_time': True,
                'yaml_filename': map_file,
            }],
            output='screen',
        ),

        # 2. AMCL — localises robot on the static map
        
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[
        params_file,
        {'use_sim_time': True},   
    ],
            output='screen',
        ),

        # 3. lifecycle manager for map_server + amcl
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
            output='screen',
        ),

        # 4. relay nav2 /cmd_vel -> /cmd_vel_raw for the converter
        Node(
            package='rover_gazebosim',
            executable='cmd_vel_relay',
            name='cmd_vel_relay',
            output='screen',
        ),

        # 5. nav2 navigation stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': 'true',
                'autostart': 'true',
            }.items(),
        ),
    ])