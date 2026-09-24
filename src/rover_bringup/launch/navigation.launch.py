"""Nav2 navigation stack for the rover.

Starts planning, control, recovery and lifecycle management only. Localisation is
*not* started here: RTAB-Map (rtabmap.launch.py) publishes /map and the map->odom
transform, in mapping mode or, with localization:=true, against a saved database.

Node list and the cmd_vel remap chain are copied from nav2_bringup's
navigation_launch.py (Nav2 1.3.13) rather than included from it: nav2_bringup
itself pulls in Gazebo, RViz and slam_toolbox as hard package.xml dependencies
(it is a demo/tutorial package, not just a library of launch files), which this
package has no use for. Composition (use_composition) is dropped for the same
reason it is unused upstream on this rover — single-process Nav2 is the norm.

    ros2 launch rover_bringup navigation.launch.py              # real rover
    ros2 launch rover_bringup navigation.launch.py sim:=true    # Gazebo / Unity
"""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_nav2')

    namespace = LaunchConfiguration('namespace')
    sim = LaunchConfiguration('sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    # sim:=true swaps in the tuning carried over from the 2-D robot workspace,
    # which reads a LaserScan instead of the fused cloud.
    default_params_file = PathJoinSubstitution([
        pkg_share,
        'config',
        PythonExpression(
            ["'nav2_params_sim.yaml' if '", sim, "'.lower() in ('true', '1') "
             "else 'nav2_params.yaml'"]
        ),
    ])

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={'autostart': autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'route_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
        'docking_server',
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Namespace to launch the Nav2 stack into'),
        DeclareLaunchArgument(
            'sim', default_value='false',
            description='Run against a simulator: selects the sim params file '
                        'and the simulated clock'),
        DeclareLaunchArgument(
            'use_sim_time', default_value=sim,
            description='Use /clock instead of wall time. Follows sim unless set'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params_file,
            description='Nav2 parameters file. Overrides the sim/real choice'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Have the lifecycle manager activate the Nav2 nodes on startup'),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            # Raw velocity, before smoothing/collision-checking. See README.md.
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=remappings,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=remappings,
        ),
        Node(
            package='nav2_route',
            executable='route_server',
            name='route_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=remappings,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=remappings,
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=remappings,
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            # Input renamed to match controller_server/behavior_server's output
            # above; output stays the hardcoded "cmd_vel_smoothed" that
            # collision_monitor's cmd_vel_in_topic parameter expects.
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=remappings,
        ),
        Node(
            package='opennav_docking',
            executable='opennav_docking',
            name='docking_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=remappings,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
            }],
        ),
    ])
