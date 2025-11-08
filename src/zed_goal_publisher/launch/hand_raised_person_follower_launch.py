#!/usr/bin/env python3

"""
Hand Raised Person Follower Launch File

このlaunchファイルは手を上げた人を追従するノードを起動します。

起動方法:
  ros2 launch zed_goal_publisher hand_raised_person_follower_launch.py

動作:
  1. ZED2iの骨格検出から複数人を認識
  2. 肩より上に手を上げている人を検出
  3. 検出した人の追跡ID（label_id）を記録
  4. その人だけを継続的に追従（距離と角度を維持）
  5. 追従対象のTFフレーム（target_person）を配信

確認方法:
  # 追跡状態の確認
  ros2 topic echo /hand_follower/status

  # ターゲットIDの確認
  ros2 topic echo /hand_follower/target_id

  # 速度指令の確認
  ros2 topic echo /cmd_vel

  # TFツリーの確認
  ros2 run tf2_tools view_frames
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch引数の宣言
    skeleton_topic_arg = DeclareLaunchArgument(
        'skeleton_topic',
        default_value='zed/zed_node/body_trk/skeletons',
        description='ZED骨格検出トピック名'
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='cmd_vel',
        description='速度指令トピック名'
    )

    target_distance_arg = DeclareLaunchArgument(
        'target_distance',
        default_value='1.5',
        description='目標追従距離 [m]'
    )

    distance_tolerance_arg = DeclareLaunchArgument(
        'distance_tolerance',
        default_value='0.4',
        description='距離許容誤差 [m]'
    )

    angle_tolerance_arg = DeclareLaunchArgument(
        'angle_tolerance',
        default_value='0.1',
        description='角度許容誤差 [rad]'
    )

    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='0.50',
        description='最大前後速度 [m/s]'
    )

    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='1.2',
        description='最大角速度 [rad/s]'
    )

    hand_raise_threshold_arg = DeclareLaunchArgument(
        'hand_raise_threshold',
        default_value='0.3',
        description='手上げ判定閾値 [m] (肩と手の高低差)'
    )

    tracking_timeout_arg = DeclareLaunchArgument(
        'tracking_timeout',
        default_value='4.0',
        description='追跡タイムアウト [s]'
    )

    # ノード定義
    hand_raised_person_follower_node = Node(
        package='zed_goal_publisher',
        executable='hand_raised_person_follower',
        name='hand_raised_person_follower',
        output='screen',
        parameters=[{
            'skeleton_topic': LaunchConfiguration('skeleton_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'target_distance': LaunchConfiguration('target_distance'),
            'distance_tolerance': LaunchConfiguration('distance_tolerance'),
            'angle_tolerance': LaunchConfiguration('angle_tolerance'),
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
            'hand_raise_threshold': LaunchConfiguration('hand_raise_threshold'),
            'tracking_timeout': LaunchConfiguration('tracking_timeout'),
        }],
        remappings=[
            # 必要に応じてリマッピング追加
        ]
    )

    return LaunchDescription([
        # Launch引数
        skeleton_topic_arg,
        cmd_vel_topic_arg,
        target_distance_arg,
        distance_tolerance_arg,
        angle_tolerance_arg,
        max_linear_speed_arg,
        max_angular_speed_arg,
        hand_raise_threshold_arg,
        tracking_timeout_arg,

        # ノード
        hand_raised_person_follower_node,
    ])
