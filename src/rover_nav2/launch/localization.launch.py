"""AMCL localisation against a saved map.

Only for runs where rover_slam is not building the map live — replaying a bag,
or driving a course that was already mapped. In the normal pipeline rover_slam
publishes /map and the map->odom transform, and this launch file stays unused.

    ros2 launch rover_nav2 localization.launch.py map:=/path/to/map.yaml
"""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_nav2')

    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    lifecycle_nodes = ['map_server', 'amcl']

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml_file,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Namespace to launch the localisation nodes into'),
        DeclareLaunchArgument(
            'map',
            description='Full path to the map yaml file to load (required)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock instead of wall time'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml']),
            description='Parameters file holding the amcl and map_server sections'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Have the lifecycle manager activate the nodes on startup'),

        GroupAction([
            PushRosNamespace(namespace),

            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[configured_params],
            ),

            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[configured_params],
            ),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': lifecycle_nodes,
                }],
            ),
        ]),
    ])
