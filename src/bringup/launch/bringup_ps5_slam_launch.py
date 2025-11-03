import launch
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
        # Declare launch arguments
        use_sim_time = LaunchConfiguration('use_sim_time', default='false')
        slam_type = LaunchConfiguration('slam_type', default='slam_toolbox')

        bringup_dir = get_package_share_directory('bringup')   # package name having a launch file
        included_launch1 = launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                        bringup_dir + '/launch/bringup_ps5_jetson_launch.py'))

        # Cartographer SLAM
        cartographer_launch = launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                        bringup_dir + '/launch/cartographer.launch.py'),
                launch_arguments={'use_sim_time': use_sim_time, 'configuration_basename': 'cartographer_scan_only.lua'}.items(),
                condition=IfCondition(PythonExpression(["'", slam_type, "' == 'cartographer'"]))
        )

        # SLAM Toolbox
        slam_toolbox_dir = get_package_share_directory('slam_toolbox')
        slam_params_file = os.path.join(
                get_package_share_directory('bringup'),
                'config',
                'mapper_params_online_async.yaml'  # 高精度地図生成版
        )

        slam_toolbox_log = launch.actions.LogInfo(
                msg=['Starting SLAM Toolbox with params file: ', slam_params_file],
                condition=IfCondition(PythonExpression(["'", slam_type, "' == 'slam_toolbox'"]))
        )

        slam_toolbox_launch = launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                        slam_toolbox_dir + '/launch/online_async_launch.py'),
                launch_arguments={
                        'use_sim_time': use_sim_time,
                        'slam_params_file': slam_params_file
                }.items(),
                condition=IfCondition(PythonExpression(["'", slam_type, "' == 'slam_toolbox'"]))
        )

        nav_dir = get_package_share_directory('nav2_bringup')
        included_launch3 = launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                        nav_dir + '/launch/my_navigation_launch.py'),
        )

        # ZED Goal Publisher Launch
        zed_goal_publisher_dir = get_package_share_directory('zed_goal_publisher')
        zed_goal_publisher_launch = launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                        zed_goal_publisher_dir + '/launch/zed_goal_publisher.launch.py'),
        )

        return LaunchDescription([

        DeclareLaunchArgument(
                'slam_type',
                default_value='slam_toolbox',
                description='SLAM type: cartographer or slam_toolbox'
        ),

        DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulation time'
        ),

        launch.actions.LogInfo(
                msg="Start zed2i, Cartographer SLAM (Scan-Matching Only Mode), navigation"
        ),

        zed_goal_publisher_launch,

        # TF-Topic同期コーディネーター（時刻同期問題解決）
        Node(
                package='bringup',
                executable='tf_sync_coordinator',
                name='tf_sync_coordinator',
                output='screen',
                parameters=[{
                    'sync_tolerance_sec': 5.0,
                    'max_sync_attempts': 10,
                    'recovery_timeout_sec': 30.0,
                    'base_frame': 'zed_camera_link',
                    'odom_frame': 'zed_odom',
                    'map_frame': 'map'
                }],
        ),

        # include launch files set the above
        included_launch1,
        cartographer_launch,
        slam_toolbox_log,
        slam_toolbox_launch,
        #emcl2_launch,
        #included_launch3

])