#!/usr/bin/env python3
# Copyright 2020 Gaitech Korea Co., Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Author: Brighten Lee

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
        output='screen',         
        parameters=[{'deadzone': 0.0}]
    )

    joy_teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',         
        parameters=[{'axis_linear.x'      : 1,
                     'axis_linear.y'      : 0,
                     'scale_linear.x'     : 0.8,
                     'scale_linear.y'     : 0.8,
                     'axis_angular.yaw'   : 3,
                     'scale_angular.yaw'  : 1.0,
                     'enable_button'      : 5,
                     'enable_turbo_button': 4
                     }],
        remappings=[('cmd_vel', '/joy_vel')],
    ) 


    return LaunchDescription([
        joy_node,
        joy_teleop_node
    ])
