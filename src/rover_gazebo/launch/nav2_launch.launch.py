


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg = get_package_share_directory('rover_gazebo')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    params_file = os.path.join(pkg, 'config', 'nav2_params.yaml')

    # SLAM mode: slam_and_rover.launch.py runs slam_toolbox, which is the
    # sole publisher of map->odom. map_server + AMCL are for localizing on a
    # pre-built map and must not run at the same time as slam_toolbox, since
    # both would try to broadcast map->odom.

    # rover_kinematics_node takes a plain Twist on /cmd_vel, so nav2 publishes
    # unstamped commands (enable_stamped_cmd_vel: false in nav2_params.yaml) and
    # no twist_to_stamped conversion is needed.

    return LaunchDescription([

        # nav2 navigation stack
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