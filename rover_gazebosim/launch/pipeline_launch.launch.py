from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_name='rover_gazebosim'
    ekf_pkg_name='rover_ekf'
    # NOTE: cmd_vel_relay (/cmd_vel -> /cmd_vel_raw) + twist_to_stamped
    # (/cmd_vel_raw -> /cmd_vel) were removed: together they formed an infinite
    # feedback loop on /cmd_vel (every command echoed forever between the two
    # nodes). nav2's collision_monitor already publishes /cmd_vel, and the
    # parameter_bridge forwards /cmd_vel straight into Gazebo.

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory(ekf_pkg_name),
                'launch',
                'ekf_dummy.launch.py'
            )),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory(pkg_name),
                'launch',
                'slam_and_rover.launch.py'
            )),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory(pkg_name),
                'launch',
                'nav2_launch.launch.py'
            )),
        ),
                Node(
            package=pkg_name,
            executable='twist_to_stamped',
            name='twist_to_stamped',
            output='screen',
        ),
    ])