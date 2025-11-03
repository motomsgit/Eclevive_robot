#!/usr/bin/env python3
"""
ZED2i ZUPT Filter Launch File

ZED2iのオドメトリにZUPT（Zero-Velocity Update）フィルタを適用します。

機能:
  - IMUデータから静止状態を検出
  - Visual Odometryの暴走を抑制
  - 急激な速度変化を制限

使用方法:
  ros2 launch zed_zupt_wrapper zed_zupt_filter.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # パッケージのconfigディレクトリパス
    config_dir = os.path.join(
        get_package_share_directory('zed_zupt_wrapper'),
        'config'
    )

    # パラメータファイルパス
    params_file = os.path.join(config_dir, 'zed_zupt_params.yaml')

    return LaunchDescription([
        Node(
            package='zed_zupt_wrapper',
            executable='zed_zupt_filter_node',
            name='zed_zupt_filter_node',
            output='screen',
            parameters=[params_file],
            emulate_tty=True,
        )
    ])
