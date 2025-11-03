#!/usr/bin/env python3
"""
ZED2i + nvblox + Nav2 統合起動ファイル

このlaunchファイルは、完全な自律ナビゲーションシステムを起動します：
- ZED2iカメラ（Visual SLAM、人体骨格検出）
- NVIDIA Isaac ROS nvblox（3Dマッピング、拡張範囲14m）
- Navigation2スタック（自律ナビゲーション）
- LiDAR前後（障害物検出）
- PS5コントローラー（テレオペレーション）
- すべてのデバイス・センサー

使用方法:
  ros2 launch bringup zed2i_nvblox_nav2_launch.py

注意:
  - AMCLは使用しません（nvbloxが map -> zed_odom TFを提供）
  - 自己位置推定はZED Visual Odometry + nvbloxで実現
  - コストマップはnvbloxのESDF点群とLiDARスキャンから生成
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description"""

    # Package directories
    bringup_dir = get_package_share_directory('bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(nav2_bringup_dir, 'params', 'my_nav2_params_mppi1.yaml')
    )
    autostart = LaunchConfiguration('autostart', default='true')
    enable_visualization = LaunchConfiguration('enable_visualization', default='false')

    # ========================================
    # 1. ZED2i + nvblox + All Devices Launch
    # ========================================
    # すべてのセンサー、nvblox、デバイスを起動
    zed_nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'zed2i_nvblox_fixed.launch.py')
        ),
        launch_arguments={
            'enable_visualization': enable_visualization,
        }.items()
    )

    # ========================================
    # 2. Navigation2 Launch (without AMCL and Map Server)
    # ========================================
    # Nav2スタックを起動（AMCLとMap Serverは除外）
    # nvbloxが map -> zed_odom TFを提供するため、AMCLは不要
    nav2_launch = TimerAction(
        period=8.0,  # ZEDとnvbloxが完全に起動するまで待機
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'my_navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'autostart': autostart,
                }.items()
            )
        ]
    )

    # ========================================
    # Log Messages
    # ========================================
    startup_log = LogInfo(msg=TextSubstitution(text=
        '\n========================================\n'
        'ZED2i + nvblox + Nav2 統合システム起動\n'
        '========================================\n'
        '\n起動コンポーネント:\n'
        '  ✓ ZED2i Camera (Visual Odometry + Body Tracking)\n'
        '  ✓ ZED ZUPT Filter (Odometry Stabilization)\n'
        '  ✓ nvblox (3D Mapping, Extended Range: 14m)\n'
        '  ✓ LiDAR Front/Back + Merger + Filter\n'
        '  ✓ PS5 Controller + Mecanum Control\n'
        '  ✓ micro-ROS Agent\n'
        '  ✓ ZED Goal Publisher\n'
        '  ✓ Safety Sensor\n'
        '  ✓ Navigation2 Stack (自律ナビゲーション)\n'
        '\n座標系構成:\n'
        '  map (nvblox global_frame)\n'
        '    └─ odom (ZED2i map_frame - Visual SLAM親フレーム)\n'
        '        └─ zed_camera_origin (ZED2i odometry_frame)\n'
        '            └─ zed_camera_link (ロボット基準)\n'
        '                ├─ base_link\n'
        '                ├─ front_lidar\n'
        '                └─ back_lidar\n'
        '\n自己位置推定:\n'
        '  - AMCLは使用しません\n'
        '  - ZED Visual Odometryで高精度な位置推定\n'
        '  - ZED2i設定: map_frame: odom, odometry_frame: zed_camera_origin\n'
        '  - map → odom: static_transform_publisher（固定、0 0 0）\n'
        '\nコストマップソース:\n'
        '  - Global: nvblox ESDF点群 + LiDARスキャン\n'
        '  - Local: ZEDポイントクラウド + LiDARスキャン\n'
        '========================================\n'
    ))

    status_log = TimerAction(
        period=10.0,
        actions=[LogInfo(msg=TextSubstitution(text=
            '\n========================================\n'
            'システム起動完了！\n'
            '========================================\n'
            '\n確認コマンド:\n'
            '  # TF確認\n'
            '  ros2 run tf2_tools view_frames\n'
            '  ros2 run tf2_ros tf2_echo map zed_camera_link\n'
            '\n  # トピック確認\n'
            '  ros2 topic hz /nvblox_node/esdf_pointcloud\n'
            '  ros2 topic hz /merged_scan_filtered\n'
            '  ros2 topic hz /cmd_vel\n'
            '\n  # ノード確認\n'
            '  ros2 node list | grep -E "nvblox|controller|planner"\n'
            '\n  # コストマップ確認\n'
            '  ros2 topic echo /global_costmap/costmap --once\n'
            '  ros2 topic echo /local_costmap/costmap --once\n'
            '\nナビゲーション開始方法:\n'
            '  1. RViz2でゴールを設定\n'
            '  2. または、ZED Goal Publisherでジェスチャー指示\n'
            '  3. または、/goal_poseトピックに直接publish\n'
            '========================================\n'
        ))]
    )

    # Create launch description
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(nav2_bringup_dir, 'params', 'my_nav2_params_mppi1.yaml'),
            description='Full path to the ROS2 parameters file to use'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),
        DeclareLaunchArgument(
            'enable_visualization',
            default_value='false',
            description='Enable RViz2 visualization',
            choices=['true', 'false']
        ),

        # Launch components
        startup_log,
        zed_nvblox_launch,
        nav2_launch,
        status_log,
    ])
