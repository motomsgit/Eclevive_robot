import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

import os
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    share_dir = get_package_share_directory('bringup')
    rviz_config_file = os.path.join(share_dir, 'config','cartographer.rviz')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', 
                                                    default='/home/jetros/ros2_ws/src/bringup/config')
    configuration_basename = LaunchConfiguration('configuration_basename', default='cartographer.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='0.5')
    return LaunchDescription([

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            output='log',
            parameters=[{
                'use_sim_time': use_sim_time,
                'lookup_transform_timeout_sec': 10.0,
                'transform_publish_period_sec': 0.02,
                'map_frame': 'map',
                'tracking_frame': 'zed_camera_link',
                'published_frame': 'zed_odom',
                'odom_frame': 'zed_odom',
                'provide_odom_frame': False,
                'use_odometry': True,
                'publish_frame_projected_to_2d': True
            }],
            remappings=[('scan', '/scan_filtered'),
                        ('odom', '/zed/zed_node/odom')],
            #remappings=[('scan', 'scan_filtered'),
            #            ('imu','zed/zed_node/imu/data')],
            arguments=['-configuration_directory', cartographer_config_dir, '-configuration_basename', configuration_basename]
            ),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
            ),

    ])