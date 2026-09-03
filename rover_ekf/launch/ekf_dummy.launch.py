# # from launch import LaunchDescription
# # from launch.actions import DeclareLaunchArgument
# # from launch.substitutions import LaunchConfiguration
# # from ament_index_python.packages import get_package_share_directory
# # import launch_ros.actions
# # import os

# # def generate_launch_description():
# #     # autostart = DeclareLaunchArgument(
# #     #     'autostart',
# #     #     default_value='true',
# #     #     description='Automatically configure and activate the node. Set to false for managed lifecycle control.')

# #     return LaunchDescription([
# #         autostart,
# #         launch_ros.actions.LifecycleNode(
# #             package='robot_localization',
# #             executable='ekf_node',
# #             name='ekf_filter_node',
# #             namespace='',
# #             output='screen',
# #             autostart=LaunchConfiguration('autostart'),
# #             parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'ekf.yaml')],
# #            ),
# # ])

# from launch import LaunchDescription
# from launch_ros.actions import Node
# import os
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='rover_ekf',
#             executable='dummy_odom_publisher',
#             name='dummy_odom_publisher',
#             output='screen',
#         ),
#         Node(
#             package='rover_ekf',
#             executable='dummy_imu_publisher',
#             name='dummy_imu_publisher',
#             output='screen',
#         ),
#         Node(
#             package='robot_localization',
#             executable='ekf_node',
#             name='ekf_filter_node',
#             output='screen',
#             parameters=[os.path.join(get_package_share_directory("rover_ekf"), 'config', 'ekf_dummy.yaml')],
#         ),
#         Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         name='map_to_odom_tf',
#         arguments=['0','0','0','0','0','0','map','odom'],
#         output='screen',
#         parameters=[{'use_sim_time':True}],
#         ),


#     ])

    



from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_name = 'rover_ekf' 
    
    ekf_config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'ekf_rover.yaml'
    )
    rover_launch_path = os.path.join(
        get_package_share_directory('rover_gazebosim'),
        'launch',
        'spawn_rover.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
             PythonLaunchDescriptionSource(rover_launch_path)
        ),

        # 2. Start the EKF node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config_path,{'use_sim_time' : True}],
        ),
        
        # 3. Static transform from map to odom (zero offset/rotation)
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        #     parameters=[{'use_sim_time':True}]
        # ),

        Node(
            package='rover_ekf',  
            executable='covariance_relay',
            name='covariance_relay',
            output='screen',
            parameters=[{'use_sim_time':True}],
        ),
        Node(
            package='rover_ekf',
            executable='wheel_splitter',
            name='wheel_splitter',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        
        # Per-wheel noise-injecting relay nodes. Each is a standalone script
        # so noise can be tuned per-wheel independently for testing.
        Node(
            package='rover_ekf',
            executable='wheel_fl',
            name='wheel_fl',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rover_ekf',
            executable='wheel_fr',
            name='wheel_fr',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rover_ekf',
            executable='wheel_rl',
            name='wheel_rl',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rover_ekf',
            executable='wheel_rr',
            name='wheel_rr',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])