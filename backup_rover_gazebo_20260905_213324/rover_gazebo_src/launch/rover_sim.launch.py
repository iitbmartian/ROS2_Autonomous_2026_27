"""Bring up the rover in Gazebo, with ros2_control and the driving node.

    ros2 launch rover_gazebo rover_sim.launch.py
    ros2 launch rover_gazebo rover_sim.launch.py world:=bars.sdf
    ros2 launch rover_gazebo rover_sim.launch.py coupling:=controller

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
                              description="file in worlds/: flat.sdf, ledge.sdf, bars.sdf"),
        DeclareLaunchArgument("coupling", default_value="mimic",
                              description="'mimic' for solver-level constraints, "
                                          "'controller' for the torque-applied fallback"),
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
        DeclareLaunchArgument(
            "spawn_from", default_value="topic",
            description="'topic' spawns the URDF that robot_state_publisher holds. "
                        "'sdf' spawns a file written by tools/make_sdf.py, which is the "
                        "way to guarantee the mimic constraints reach the solver."),
        DeclareLaunchArgument(
            "sdf_file",
            default_value=os.path.join(share, "models", "rover", "model.sdf")),
    ]

    coupling = LaunchConfiguration("coupling")
    controllers_file = os.path.join(share, "config", "controllers.yaml")
    control_params = os.path.join(share, "config", "rover_control.yaml")

    robot_description = ParameterValue(Command([
        "xacro ", PathJoinSubstitution([FindPackageShare(PKG), "urdf", "rover.urdf.xacro"]),
        " coupling:=", coupling,
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

    # Either source of the model gives the same joint names, which is all that
    # gz_ros2_control needs to match its hardware interface against.
    spawn_source = PythonExpression(
        ["'-file' if '", LaunchConfiguration("spawn_from"), "' == 'sdf' else '-topic'"])
    spawn_value = PythonExpression(
        ["'", LaunchConfiguration("sdf_file"), "' if '",
         LaunchConfiguration("spawn_from"), "' == 'sdf' else 'robot_description'"])

    spawn = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=[spawn_source, spawn_value, "-name", "rover",
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
    # Only under the fallback: the mimic path leaves the rockers passive.
    fallback = IfCondition(PythonExpression(["'", coupling, "' == 'controller'"]))

    coupling_ctl = Node(
        package="controller_manager", executable="spawner", output="screen",
        arguments=["rocker_coupling_controller", "--controller-manager", "/controller_manager"],
        condition=fallback,
    )

    kinematics = Node(
        package=PKG, executable="rover_kinematics_node.py", name="rover_kinematics",
        output="screen", parameters=[control_params],
    )

    coupling_node = Node(
        package=PKG, executable="rocker_coupling_node.py", name="rocker_coupling",
        output="screen", parameters=[control_params],
        condition=fallback,
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
