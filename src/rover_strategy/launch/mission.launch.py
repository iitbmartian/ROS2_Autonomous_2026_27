from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_strategy',
            executable='mission_commander',
            name='mission_commander',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
