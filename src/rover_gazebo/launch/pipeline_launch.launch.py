"""Full stack on the rover_gazebo rover: Gazebo + EKF, SLAM, nav2 and the mission node.

rover_gazebo counterpart of rover_gazebosim/launch/pipeline_launch.launch.py.

    ros2 launch rover_gazebo pipeline_launch.launch.py
    ros2 launch rover_gazebo pipeline_launch.launch.py world:=bars.sdf
"""

from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_name = 'rover_gazebo'
    ekf_pkg_name = 'rover_ekf'
    strategy_pkg_name = 'rover_strategy'
    # No twist_to_stamped here: nav2 publishes a plain Twist on /cmd_vel, which is
    # exactly what rover_kinematics_node subscribes to.

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty_world.sdf',
                              description='file in rover_gazebo/worlds: empty_world.sdf, flat.sdf, ledge.sdf, bars.sdf'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_imu', default_value='false',
                              description='also fuse the gyro yaw rate into the EKF'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory(ekf_pkg_name),
                'launch',
                'ekf_rover_gazebo.launch.py'
            )),
            launch_arguments={'world': LaunchConfiguration('world'),
                              'gui': LaunchConfiguration('gui'),
                              'use_imu': LaunchConfiguration('use_imu')}.items(),
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory(strategy_pkg_name),
                'launch',
                'mission.launch.py'
            )),
        ),
    ])
