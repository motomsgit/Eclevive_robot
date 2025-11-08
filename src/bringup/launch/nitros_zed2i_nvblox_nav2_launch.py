#!/usr/bin/env python3
"""
ZED2i + nvblox + Nav2 統合起動ファイル (NITROS対応版)

このlaunchファイルは、NITROS (Isaac ROS高速通信)を有効活用できる完全な自律ナビゲーションシステムを起動します：
- ZED2iカメラ（Visual SLAM、人体骨格検出）with NITROS
- NVIDIA Isaac ROS nvblox（3Dマッピング、拡張範囲14m）with NITROS
- Navigation2スタック（自律ナビゲーション）
- LiDAR前後（障害物検出）
- PS5コントローラー（テレオペレーション）
- すべてのデバイス・センサー

使用方法:
  # NITROS有効（デフォルト、推奨）
  ros2 launch bringup nitros_zed2i_nvblox_nav2_launch.py

  # IPC有効（NITROS無効、従来方式）
  ros2 launch bringup nitros_zed2i_nvblox_nav2_launch.py enable_ipc:=true

  # 可視化付き
  ros2 launch bringup nitros_zed2i_nvblox_nav2_launch.py enable_visualization:=true

注意:
  - デフォルトでNITROS有効（enable_ipc:=false）
  - enable_ipc:=false でNITROS有効化（Isaac ROS高速通信、省略可）
  - enable_ipc:=true で従来のIPC通信（互換性優先）
  - NITROSはZED→nvblox間の画像転送を高速化（GPU Direct）
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
    enable_ipc = LaunchConfiguration('enable_ipc', default='false')

    # ========================================
    # 1. ZED2i + nvblox + All Devices Launch (with enable_ipc parameter)
    # ========================================
    # すべてのセンサー、nvblox、デバイスを起動（NITROS対応）
    zed_nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'nitros_zed2i_nvblox_fixed.launch.py')
        ),
        launch_arguments={
            'enable_visualization': enable_visualization,
            'enable_ipc': enable_ipc,
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
        '(NITROS対応版)\n'
        '========================================\n'
        '\n起動コンポーネント:\n'
        '  ✓ ZED2i Camera (Visual Odometry + Body Tracking) [NITROS]\n'
        '  ✓ ZED ZUPT Filter (Odometry Stabilization)\n'
        '  ✓ nvblox (3D Mapping, Extended Range: 14m) [NITROS]\n'
        '  ✓ LiDAR Front/Back + Merger + Filter\n'
        '  ✓ PS5 Controller + Mecanum Control\n'
        '  ✓ micro-ROS Agent\n'
        '  ✓ ZED Goal Publisher\n'
        '  ✓ Safety Sensor\n'
        '  ✓ Navigation2 Stack (自律ナビゲーション)\n'
        '    - Collision Monitor (障害物回避強化)\n'
        '      - Stop Zone: 0.25m (緊急停止)\n'
        '      - Slowdown Zone: 0.45m (40%減速)\n'
        '      - Approach Zone: 1.5秒先予測\n'
        '\nNITROS設定:\n'
        '  - enable_ipc=false: NITROS有効（Isaac ROS高速通信）\n'
        '  - enable_ipc=true: IPC有効（従来方式）\n'
        '  - ZED→nvblox間の画像転送を高速化（GPU Direct）\n'
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
            'システム起動完了！(NITROS版)\n'
            '========================================\n'
            '\n確認コマンド:\n'
            '  # TF確認\n'
            '  ros2 run tf2_tools view_frames\n'
            '  ros2 run tf2_ros tf2_echo map zed_camera_link\n'
            '\n  # トピック確認\n'
            '  ros2 topic hz /nvblox_node/esdf_pointcloud\n'
            '  ros2 topic hz /scan_filtered\n'
            '  ros2 topic hz /cmd_vel\n'
            '\n  # ノード確認\n'
            '  ros2 node list | grep -E "nvblox|controller|planner|collision"\n'
            '\n  # コストマップ確認\n'
            '  ros2 topic echo /global_costmap/costmap --once\n'
            '  ros2 topic echo /local_costmap/costmap --once\n'
            '\n  # Collision Monitor確認\n'
            '  ros2 topic hz /cmd_vel_monitored\n'
            '  ros2 topic echo /polygon_stop\n'
            '  ros2 topic echo /polygon_slowdown\n'
            '\n  # NITROS動作確認\n'
            '  ros2 topic info /zed/zed_node/rgb/image_rect_color/nitros\n'
            '  ros2 topic info /zed/zed_node/depth/depth_registered/nitros\n'
            '\nナビゲーション開始方法:\n'
            '  1. RViz2でゴールを設定\n'
            '  2. または、ZED Goal Publisherでジェスチャー指示\n'
            '  3. または、/goal_poseトピックに直接publish\n'
            '\n障害物回避:\n'
            '  - Collision Monitorが自動的に障害物を検出\n'
            '  - 0.25m以内: 緊急停止\n'
            '  - 0.45m以内: 40%減速\n'
            '  - 1.5秒先: 動的予測回避\n'
            '\nNITROSパフォーマンス:\n'
            '  - GPU Direct通信により画像転送が高速化\n'
            '  - CPU負荷が低減\n'
            '  - レイテンシが改善\n'
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
        DeclareLaunchArgument(
            'enable_ipc',
            default_value='false',
            description='Enable IPC (Intra-Process Communication). Set to false for NITROS (recommended)',
            choices=['true', 'false']
        ),

        # Launch components
        startup_log,
        zed_nvblox_launch,
        nav2_launch,
        status_log,
    ])
