"""Bring up the rover in Gazebo, with ros2_control and the driving node.

    ros2 launch rover_gazebo rover_sim.launch.py
    ros2 launch rover_gazebo rover_sim.launch.py world:=bars.sdf

Then drive it:

    ros2 run rover_gazebo teleop_rover.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PKG = "rover_gazebo"


def generate_launch_description():
    share = get_package_share_directory(PKG)

    args = [
        DeclareLaunchArgument("world", default_value="flat.sdf",
                              description="file in worlds/: flat.sdf, ledge.sdf, bars.sdf, "
                                          "husarion_world.sdf"),
        DeclareLaunchArgument("sim", default_value="harmonic",
                              description="'harmonic' or 'fortress'; picks plugin names"),
        DeclareLaunchArgument("mass_scale", default_value="1.0"),
        DeclareLaunchArgument("mesh_collision", default_value="false"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("teleop", default_value="false",
                              description="launch keyboard teleop in this terminal"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.02"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
    ]

    controllers_file = os.path.join(share, "config", "controllers.yaml")
    control_params = os.path.join(share, "config", "rover_control.yaml")

    robot_description = ParameterValue(Command([
        "xacro ", PathJoinSubstitution([FindPackageShare(PKG), "urdf", "rover.urdf.xacro"]),
        " sim:=", LaunchConfiguration("sim"),
        " mass_scale:=", LaunchConfiguration("mass_scale"),
        " mesh_collision:=", LaunchConfiguration("mesh_collision"),
        " controllers_file:=", controllers_file,
    ]), value_type=str)

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
        launch_arguments={
            "gz_args": [
                PathJoinSubstitution([FindPackageShare(PKG), "worlds",
                                      LaunchConfiguration("world")]),
                " -r",
                # gui:=false runs the server alone, which is what the test harness wants
                PythonExpression(["'' if '", LaunchConfiguration("gui"),
                                  "'.lower() in ('true', '1') else ' -s --headless-rendering'"]),
            ],
            "on_exit_shutdown": "true",
        }.items(),
    )

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    spawn = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "robot_description", "-name", "rover",
                   "-x", LaunchConfiguration("x"),
                   "-y", LaunchConfiguration("y"),
                   "-z", LaunchConfiguration("z"),
                   "-Y", LaunchConfiguration("yaw")],
    )

    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        parameters=[{"config_file": os.path.join(share, "config", "bridge.yaml"),
                     "use_sim_time": True}],
    )

    def spawner(name, *extra):
        return Node(package="controller_manager", executable="spawner", output="screen",
                    arguments=[name, "--controller-manager", "/controller_manager", *extra])

    jsb = spawner("joint_state_broadcaster")
    steer = spawner("steer_controller")
    wheel = spawner("wheel_controller")
    coupling_ctl = spawner("rocker_coupling_controller")

    kinematics = Node(
        package=PKG, executable="rover_kinematics_node.py", name="rover_kinematics",
        output="screen", parameters=[control_params],
    )

    coupling_node = Node(
        package=PKG, executable="rocker_coupling_node.py", name="rocker_coupling",
        output="screen", parameters=[control_params],
    )

    teleop = ExecuteProcess(
        cmd=["ros2", "run", PKG, "teleop_rover.py"],
        output="screen", condition=IfCondition(LaunchConfiguration("teleop")),
    )

    # Controllers can only be spawned once the model, and with it the controller
    # manager, exists inside the simulator.
    after_spawn = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb]))
    after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb,
                      on_exit=[steer, wheel, coupling_ctl, kinematics, coupling_node, teleop]))

    return LaunchDescription(args + [gz, rsp, bridge, spawn, after_spawn, after_jsb])
