import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_gazebo')
    urdf_path = os.path.join(pkg_share, 'urdf', 'rover.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'khali.sdf')
    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r ' + world_path}.items()
    )

    bridge_config = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_model',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'rover',
            '-z', '0.02'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim_launch,
        ros_gz_bridge,
        robot_state_publisher,
        spawn_model,
    ])
