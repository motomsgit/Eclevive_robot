import launch
import os
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_file = LaunchConfiguration('map_yaml_file', default='')
    use_localization = LaunchConfiguration('use_localization', default='true')

    # Get package directories
    bringup_dir = get_package_share_directory('bringup')
    nav_dir = get_package_share_directory('nav2_bringup')

    # EMCL2 parameters file
    emcl2_params_file = os.path.join(
        get_package_share_directory('bringup'),
        'config',
        'emcl2_params.yaml'
    )

    # Include bringup_ps5_jetson_launch.py (sensors, devices)
    included_launch_devices = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            bringup_dir + '/launch/bringup_ps5_jetson_launch.py'))

    # Include localization_emcl_launch.py with EMCL2
    # Use TimerAction to delay localization start, waiting for ZED TF frames
    localization_launch = TimerAction(
        period=8.0,  # Wait 8 seconds for ZED camera to initialize and publish TF frames
        actions=[
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    nav_dir + '/launch/localization_emcl_launch.py'),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': map_yaml_file,
                    'params_file': emcl2_params_file,
                    'autostart': 'true',
                    'use_composition': 'False',
                    'use_respawn': 'False',
                }.items()
            )
        ]
    )

    # Include navigation launch
    navigation_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            nav_dir + '/launch/my_navigation_launch.py'),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # ZED Goal Publisher Launch
    zed_goal_publisher_dir = get_package_share_directory('zed_goal_publisher')
    zed_goal_publisher_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            zed_goal_publisher_dir + '/launch/zed_goal_publisher.launch.py'),
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'map_yaml_file',
            default_value='',
            description='Full path to map yaml file to load'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            'use_localization',
            default_value='true',
            description='Whether to use localization or not'
        ),

        launch.actions.LogInfo(
            msg="Starting ZED2i, EMCL2 Localization, and Navigation"
        ),

        # ZED Goal Publisher
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

        # Include launch files
        included_launch_devices,
        localization_launch,
        #navigation_launch,  

    ])
