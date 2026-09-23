"""Bring up the rover in Gazebo, with ros2_control and the driving node.

    ros2 launch rover_gazebo rover_sim.launch.py
    ros2 launch rover_gazebo rover_sim.launch.py world:=bars.sdf

Then drive it:

    ros2 run rover_gazebo teleop_rover.py

The suspension's closed kinematic loops are held by constraint torque by default
(rocker_coupling_node.py), which works on every engine. An alternative, real <mimic>
joints plus a much simpler single-joint hold (rocker_hold_node.py), is available but
Bullet-Featherstone only -- DART silently ignores <mimic> and would leave the
suspension uncontrolled, so the two arguments below must be set together:

    ros2 launch rover_gazebo rover_sim.launch.py suspension:=mimic \\
        physics_engine:=gz-physics-bullet-featherstone-plugin

See rover.urdf.xacro's own docstring and doc/VERIFICATION.md for the measured
difference between the two.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
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
        DeclareLaunchArgument("sim", default_value="harmonic",
                              description="'harmonic' or 'fortress'; picks plugin names"),
        DeclareLaunchArgument("physics_engine", default_value="",
                              description="gz sim --physics-engine value, e.g. "
                                          "gz-physics-bullet-featherstone-plugin; "
                                          "empty keeps gz sim's own default (DART)"),
        DeclareLaunchArgument("suspension", default_value="torque",
                              description="'torque' (default, every engine incl. "
                                          "DART): rocker_coupling_node holds all "
                                          "four rockers by constraint torque. "
                                          "'mimic' (Bullet-Featherstone only, pair "
                                          "with physics_engine:=gz-physics-bullet-"
                                          "featherstone-plugin): BLS/BRS/FRS are "
                                          "real <mimic> joints, rocker_hold_node "
                                          "holds only FLS. DART silently ignores "
                                          "<mimic> and leaves the suspension "
                                          "uncontrolled -- do not use 'mimic' "
                                          "without also setting physics_engine."),
        DeclareLaunchArgument("mass_scale", default_value="1.0"),
        DeclareLaunchArgument("mesh_collision", default_value="false"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("teleop", default_value="false",
                              description="launch keyboard teleop in this terminal"),
        DeclareLaunchArgument("teleop_gui", default_value="false",
                              description="launch the Tkinter keyboard teleop window"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.02"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
    ]

    # suspension:=mimic needs controller_manager to load the fewer-joints
    # rocker_coupling_controller block (config/controllers_mimic.yaml) instead of
    # the four-joint default -- see rover_ros2_control.xacro. Resolved at runtime
    # (PythonExpression), since LaunchConfiguration("suspension") isn't known yet
    # while this launch description is only being built.
    controllers_file = PythonExpression([
        "'", os.path.join(share, "config", "controllers_mimic.yaml"), "' if '",
        LaunchConfiguration("suspension"), "' == 'mimic' else '",
        os.path.join(share, "config", "controllers.yaml"), "'",
    ])
    control_params = os.path.join(share, "config", "rover_control.yaml")

    robot_description = ParameterValue(Command([
        "xacro ", PathJoinSubstitution([FindPackageShare(PKG), "urdf", "rover.urdf.xacro"]),
        " sim:=", LaunchConfiguration("sim"),
        " mass_scale:=", LaunchConfiguration("mass_scale"),
        " mesh_collision:=", LaunchConfiguration("mesh_collision"),
        " suspension:=", LaunchConfiguration("suspension"),
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
                PythonExpression(["'' if '", LaunchConfiguration("physics_engine"),
                                  "' == '' else ' --physics-engine ' + '",
                                  LaunchConfiguration("physics_engine"), "'"]),
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
    # Same controller name either way; which joints it actually claims differs
    # between config/controllers.yaml (all four rockers) and
    # config/controllers_mimic.yaml (FLS only) -- see the controllers_file
    # PythonExpression above.
    coupling_ctl = spawner("rocker_coupling_controller")
    # explicit_pid mode's (teleop key 5) controllers. Loaded and configured but left
    # inactive -- steer_controller/wheel_controller above are what's actually driving
    # the 8 joints at startup (default mode is ackermann). rover_kinematics_node
    # activates these two and deactivates those two, and vice versa, whenever the
    # mode crosses the explicit/explicit_pid boundary; see its on_mode().
    steer_pid = spawner("steer_pid_controller", "--inactive")
    wheel_pid = spawner("wheel_pid_controller", "--inactive")

    kinematics = Node(
        package=PKG, executable="rover_kinematics_node.py", name="rover_kinematics",
        output="screen", parameters=[control_params],
    )

    # Exactly one of these two runs, chosen by suspension:= -- see the
    # DeclareLaunchArgument above for what each does and why "mimic" needs
    # physics_engine:=gz-physics-bullet-featherstone-plugin alongside it.
    suspension_is_mimic = PythonExpression(
        ["'", LaunchConfiguration("suspension"), "' == 'mimic'"])
    coupling_node = Node(
        package=PKG, executable="rocker_coupling_node.py", name="rocker_coupling",
        output="screen", parameters=[control_params],
        condition=UnlessCondition(suspension_is_mimic),
    )
    hold_node = Node(
        package=PKG, executable="rocker_hold_node.py", name="rocker_hold",
        output="screen", parameters=[control_params],
        condition=IfCondition(suspension_is_mimic),
    )

    # explicit_pid mode's independent per-joint PID, one loop per steer/drive joint.
    # Always launched, like coupling_node above -- harmless while
    # steer_pid_controller/wheel_pid_controller are inactive, since ros2_control
    # drops commands sent to an inactive controller.
    control_node = Node(
        package=PKG, executable="control_node.py", name="explicit_pid_control",
        output="screen", parameters=[control_params],
    )

    # The params file carries the keyboard feel, including the explicit-mode sweep
    # rate. Run teleop_rover.py by hand in a second terminal and it falls back to the
    # defaults declared in the script.
    teleop = ExecuteProcess(
        cmd=["ros2", "run", PKG, "teleop_rover.py",
             "--ros-args", "--params-file", control_params],
        output="screen", condition=IfCondition(LaunchConfiguration("teleop")),
    )

    # The Tkinter alternative: a small keyboard-focused window instead of the
    # launching terminal. Unlike teleop above it needs no TTY, just a display, so it's
    # spawned as an ordinary Node rather than ExecuteProcess. Reads real per-key
    # press/release events, so holding two keys together (e.g. W+A for an arc) is
    # reliable -- see teleop_rover_gui.py's docstring.
    teleop_gui = Node(
        package=PKG, executable="teleop_rover_gui.py", name="teleop_rover_gui",
        output="screen", parameters=[control_params],
        condition=IfCondition(LaunchConfiguration("teleop_gui")),
    )

    # Controllers can only be spawned once the model, and with it the controller
    # manager, exists inside the simulator.
    after_spawn = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb]))
    after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb,
                      on_exit=[steer, wheel, coupling_ctl, steer_pid, wheel_pid,
                               kinematics, coupling_node, hold_node, control_node,
                               teleop, teleop_gui]))

    return LaunchDescription(args + [gz, rsp, bridge, spawn, after_spawn, after_jsb])
