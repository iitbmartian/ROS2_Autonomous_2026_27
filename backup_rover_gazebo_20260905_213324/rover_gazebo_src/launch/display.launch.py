"""Show the model in RViz with no simulator, to check geometry and the frame fix.

    ros2 launch rover_gazebo display.launch.py

The joint sliders move the free suspension joint and the eight driven joints. Under
coupling:=mimic, robot_state_publisher resolves the mimic tags itself, so dragging the
FLS slider moves all four rockers and the differential bar together. That is the
quickest way to confirm the constraints are written correctly, and it works whatever
simulator you have installed.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PKG = "rover_gazebo"


def generate_launch_description():
    robot_description = ParameterValue(Command([
        "xacro ", PathJoinSubstitution([FindPackageShare(PKG), "urdf", "rover.urdf.xacro"]),
        " coupling:=", LaunchConfiguration("coupling"),
        " ros2_control:=false",
    ]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("coupling", default_value="mimic"),
        DeclareLaunchArgument("gui", default_value="true"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}], output="screen"),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             output="screen"),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", PathJoinSubstitution(
                 [FindPackageShare(PKG), "rviz", "rover.rviz"])]),
    ])
