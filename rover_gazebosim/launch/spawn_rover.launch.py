import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import xacro
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


# from launch.actions import SetParameter

def generate_launch_description():
    # Load robot description from URDF file
    common_dir_path = LaunchConfiguration("common_dir_path")
    declare_common_dir_path_arg = DeclareLaunchArgument(
        "common_dir_path",
        default_value="",
        description="Path to the common configuration directory.",
    )

    description_pkg = FindPackageShare("husarion_ugv_description")
    description_common_dir = PythonExpression(
        [
            "'",
            common_dir_path,
            "/husarion_ugv_description",
            "' if '",
            common_dir_path,
            "' else '",
            description_pkg,
            "'",
        ]
    )

    battery_config_path = LaunchConfiguration("battery_config_path")
    declare_battery_config_path_arg = DeclareLaunchArgument(
        "battery_config_path",
        description=(
            "Path to the Ignition LinearBatteryPlugin configuration file. "
            "This configuration is intended for use in simulations only."
        ),
        default_value="",
    )

    components_config_path = LaunchConfiguration("components_config_path")
    declare_components_config_path_arg = DeclareLaunchArgument(
        "components_config_path",
        default_value=PathJoinSubstitution([description_common_dir, "config", "components.yaml"]),
        description=(
            "Specify file which contains components. These components will be included in URDF."
            "Available options can be found in manuals: https://husarion.com/manuals"
        ),
    )

    use_madgwick_filter = LaunchConfiguration("use_madgwick_filter")
    declare_use_madgwick_filter_arg = DeclareLaunchArgument(
        "use_madgwick_filter",
        default_value="False",
        description="Determine orientation from IMU",
        choices=["True", "true", "False", "false"],
    )

    wheel_type = LaunchConfiguration("wheel_type")
    controller_config_path = LaunchConfiguration("controller_config_path")
    declare_controller_config_path_arg = DeclareLaunchArgument(
        "controller_config_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("husarion_ugv_controller"),
                "config",
                PythonExpression(["'", wheel_type, "_controller.yaml'"]),
            ]
        ),
        description=(
            "Path to controller configuration file. By default, it is located in"
            " 'husarion_ugv_controller/config/{wheel_type}_controller.yaml'. You can also specify"
            " the path to your custom controller configuration file here. "
        ),
    )

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    robot_model = LaunchConfiguration("robot_model")
    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable(name="ROBOT_MODEL_NAME", default_value="panther"),
        description="Specify robot model",
        choices=["lynx", "panther"],
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="True",
        description="Whether simulation is used.",
        choices=["True", "true", "False", "false"],
    )

    wheel_config_path = LaunchConfiguration("wheel_config_path")
    declare_wheel_config_path_arg = DeclareLaunchArgument(
        "wheel_config_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("husarion_ugv_description"),
                "config",
                PythonExpression(["'", wheel_type, ".yaml'"]),
            ]
        ),
        description=(
            "Path to wheel configuration file. By default, it is located in "
            "'husarion_ugv_description/config/{wheel_type}.yaml'. You can also specify the path "
            "to your custom wheel configuration file here. "
        ),
    )

    default_wheel_type = {"lynx": "WH05", "panther": "WH01"}
    declare_wheel_type_arg = DeclareLaunchArgument(
        "wheel_type",
        default_value=PythonExpression([f"{default_wheel_type}['", robot_model, "']"]),
        description=(
            "Specify the wheel type. If the selected wheel type is not 'custom', "
            "the 'wheel_config_path' and 'controller_config_path' arguments will be "
            "automatically adjusted and can be omitted."
        ),
        choices=["WH01", "WH02", "WH04", "WH05", "custom"],
    )

    ns = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])

    orientation_covariance = PythonExpression(
        [
            "[1.8e-3, 0.0, 0.0, 0.0, 1.8e-3, 0.0, 0.0, 0.0, 1.8e-3] if '",
            use_madgwick_filter,
            "' in ['True', 'true'] else ",
            "[-1.0, 0.0, 0.0, 0.0, 1.8e-3, 0.0, 0.0, 0.0, 1.8e-3]",  # the first element of the orientation covariance is set to -1 according to the documentation: https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Imu.html
        ]
    )

    ns_controller_config_path = ReplaceString(
        controller_config_path,
        {
            "<namespace>/": ns,
            "<static_covariance_orientation>": orientation_covariance,
        },
    )

    # Get URDF via xacro
    imu_pos_x = os.environ.get("ROBOT_IMU_LOCALIZATION_X", "0.168")
    imu_pos_y = os.environ.get("ROBOT_IMU_LOCALIZATION_Y", "0.028")
    imu_pos_z = os.environ.get("ROBOT_IMU_LOCALIZATION_Z", "0.083")
    imu_rot_r = os.environ.get("ROBOT_IMU_ORIENTATION_R", "3.14")
    imu_rot_p = os.environ.get("ROBOT_IMU_ORIENTATION_P", "-1.57")
    imu_rot_y = os.environ.get("ROBOT_IMU_ORIENTATION_Y", "0.0")
    urdf_file = PythonExpression(["'", robot_model, ".urdf.xacro'"])
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("husarion_ugv_description"), "urdf", urdf_file]
            ),
            " use_sim:=",
            use_sim,
            " wheel_config_file:=",
            wheel_config_path,
            " controller_config_file:=",
            ns_controller_config_path,
            " battery_config_file:=",
            battery_config_path,
            " imu_xyz:=",
            f"'{imu_pos_x} {imu_pos_y} {imu_pos_z}'",
            " imu_rpy:=",
            f"'{imu_rot_r} {imu_rot_p} {imu_rot_y}'",
            " namespace:=",
            namespace,
            " components_config_path:=",
            components_config_path,
            " use_madgwick_filter:=",
            use_madgwick_filter,
        ]
    )

    # robot_description = xacro.process_file(
    #     os.path.join(get_package_share_directory('rover_gazebosim'), 'urdf/rover.urdf')
    # ).toxml()
    robot_description=robot_description_content
    # Load configuration for parameter bridge
    config_file = os.path.join(get_package_share_directory('rover_gazebosim'), 'config', 'joint_names_mobility urdf adaptation.yaml')
    #world_file = os.path.join(get_package_share_directory('rover_gazebosim'), 'world', 'husarion_office.sdf')
    #world_file = os.path.join(get_package_share_directory('rover_gazebosim'), 'world', 'husarion_officex1.5.sdf')
    world_file = os.path.join(get_package_share_directory('rover_gazebosim'), 'world', 'husarion_world.sdf')
    # world_file = os.path.join(get_package_share_directory('rover_gazebosim'), 'world', 'warehouse.world')
    # world_file = os.path.join(get_package_share_directory('rover_gazebosim'), 'world', 'rover.world')
    # world_file = os.path.join(get_package_share_directory('rover_gazebosim'), 'world', 'new.sdf')

    pkg_sim=get_package_share_directory('ros_gz_sim')
    return LaunchDescription([
        declare_common_dir_path_arg,
        declare_battery_config_path_arg,
        declare_components_config_path_arg,
        declare_robot_model_arg,
        declare_use_madgwick_filter_arg,
        declare_wheel_type_arg,
        declare_controller_config_path_arg,
        declare_namespace_arg,
        declare_use_sim_arg,
        declare_wheel_config_path_arg,
        DeclareLaunchArgument('gui', default_value='true', description='Enable/Disable GUI'),
        DeclareLaunchArgument('x', default_value='7', description='Initial x position of the rover'),
        DeclareLaunchArgument('y', default_value='-5', description='Initial y position of the rover'),
        DeclareLaunchArgument('z', default_value='0.2', description='Initial z position of the rover'),
        
        # Set environment variables for resource paths
        # SetEnvironmentVariable(
        #     name='IGN_GAZEBO_RESOURCE_PATH',
        #     value=os.path.join(get_package_share_directory('rover_gazebosim'), 'meshes') + ':' +
        #           os.path.join(get_package_share_directory('rover_gazebosim'), 'urdf')
        # ),
#         SetEnvironmentVariable(
#     name='GZ_SIM_RESOURCE_PATH',
#     value=get_package_share_directory('rover_gazebosim')
# ),
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=os.path.join(get_package_share_directory('rover_gazebosim'), 'model') + ':' +
                  os.path.join(get_package_share_directory('rover_gazebosim'), 'models'),
        ),

        # Launch Ignition Gazebo with a specific world file
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        #     ),
        #     launch_arguments={
        #         "gz_args": "~/mrt_ws/src/rover_gazebosim/worlds/ign_rect_world.sdf"
        #     }.items(),
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_sim, "launch", "gz_sim.launch.py")),
            launch_arguments={
                # -r: start the world running (unpaused) instead of waiting for play
                "gz_args": f"-r {world_file}"}.items(),),
        # Robot State Publisher for publishing the robot's URDF to the parameter server
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{"robot_description": robot_description},{'use_sim_time': True} ]
        ),

        # # Spawn the rover in Ignition Gazebo
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            name="rover_spawn",
            arguments=[
                "-string", robot_description,
                "-name", "rover",
                "-x", LaunchConfiguration("x"),
                "-y", LaunchConfiguration("y"),
                "-z", LaunchConfiguration("z"),
            ],
        ),

        # ROS-Gazebo Bridge for parameter communication
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': config_file},{'use_sim_time': True}]
        ),
    ])



# import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from ament_index_python.packages import get_package_share_directory
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# import xacro
# from launch.substitutions import LaunchConfiguration
# def generate_launch_description():
#     # Path to the world file
#     world_file = os.path.join(get_package_share_directory('rover_gazebosim'), 'world', 'rover.sdf')
#     pkg_sim=get_package_share_directory('ros_gz_sim')
#     robot_description = xacro.process_file(
#         os.path.join(get_package_share_directory('rover_gazebosim'), 'urdf/rrbot.urdf')
#     ).toxml()

    
#     gazebo=IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 os.path.join(pkg_sim, "launch", "gz_sim.launch.py")),
#             launch_arguments={
#                 "gz_args": world_file}.items(),),Node(
#             package="ros_gz_sim",
#             executable="create",
#             output="screen",
#             name="rover_spawn",
#             arguments=[
#                 "-string", robot_description,
#                 "-name", "rover",
#                 # "-x", LaunchConfiguration("x"),
#                 # "-y", LaunchConfiguration("y"),
#                 # "-z", LaunchConfiguration("z"),
#             ],
#         )
#     config_file = os.path.join(get_package_share_directory('rover_gazebosim'), 'config', 'joint_names_mobility urdf adaptation.yaml')


                
#     return LaunchDescription([
#          DeclareLaunchArgument('gui', default_value='true', description='Enable/Disable GUI'),
#          DeclareLaunchArgument('x', default_value='0', description='Initial x position of the rover'),
#          DeclareLaunchArgument('y', default_value='0', description='Initial y position of the rover'),
#          DeclareLaunchArgument('z', default_value='0.5', description='Initial z position of the rover'),
#          IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 os.path.join(pkg_sim, "launch", "gz_sim.launch.py")),
#             launch_arguments={
#                 "gz_args": world_file}.items(),),Node(
#             package="ros_gz_sim",
#             executable="create",
#             output="screen",
#             name="rover_spawn",
#             arguments=[
#                 "-string", robot_description,
#                 "-name", "rover",
#                 "-x", LaunchConfiguration("x"),
#                 "-y", LaunchConfiguration("y"),
#                 "-z", LaunchConfiguration("z"),
#             ],),
#             Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             output='screen',
#             parameters=[{"robot_description": robot_description}]
#         ),
#         Node(
#             package='joint_state_publisher',
#             executable='joint_state_publisher',
#             name='joint_state_publisher',
#             parameters=[{'use_gui': LaunchConfiguration('gui')}]
#         ),
#         Node(
#             package='ros_gz_bridge',
#             executable='parameter_bridge',
#             parameters=[{'config_file': config_file}]
#         ),
#     ])