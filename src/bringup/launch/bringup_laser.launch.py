import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    #--------------------velodyne  -------------------------------------------------------------
    velodyne_dir = get_package_share_directory('velodyne')   # package name having a launch file
    included_launch1 = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                    velodyne_dir + '/launch/velodyne-all-nodes-VLP16-launch.py'))
    

    #--------------------ld19 launch  -------------------------------------------------------------
    ldlidar_dir = get_package_share_directory('ldlidar_stl_ros2')   # package name having a launch file
    included_launch2 = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                    ldlidar_dir + '/launch/stl27l.launch.py'))
    
    #--------------------laser_merger  -------------------------------------------------------------
    scan_dir = get_package_share_directory('laser_merger2')   # package name having a launch file
    included_launch3 = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                    scan_dir + '/launch/laser_merger_for_slam.launch.py'))
    
    #--------------------laser_filter  -------------------------------------------------------------
    filters_dir = get_package_share_directory('laser_filters')   # package name having a launch file
    included_launch4 = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                    filters_dir + '/examples/laser_box_filter_example.launch.py'))
    

    return LaunchDescription([

        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=["multiserial", "--devs", "/dev/ttyACM1 /dev/ttyACM0 /dev/M5_scale", "--baud" ,"1000000"],
            #arguments=["multiserial", "--devs", "/dev/ttyACM0 /dev/ttyUSB0", "--baud" ,"1000000"],
            #arguments=["serial", "--dev", "/dev/ttyACM1", "--baud" ,"1000000"],
            respawn=True,
            respawn_delay=5,
            output='screen'
        ),

        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux',
            output='screen',
            parameters=[
                {'dev': '/dev/input/js0',
                 'dev_name': 'DualSense Wireless Controller',
                 'deadzone': 0.05,
                 'autorepeat_rate': 30.0,
                }
            ]
        ),

        Node(
            package='joy_mecanum_controller',
            executable='joy_slam_maker_node',
            name='joy_slam_maker',
            output='screen',    
        ),

        Node(
            package='safety_sensor',
            executable='safety_sensor',
            name='safety_sensor',
            output='screen',     
        ),
       
        Node( 
            package='tf2_ros',
            executable='static_transform_publisher',
            name='velodyne_static_transform',
            arguments=['-0.18', '0.0', '0.15', '3.14159', '0.0', '0', 'base_link', 'velodyne']
    	),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_stl27_laser',
            arguments=['0.12','0','0.10', '0','0','0','base_link','stl27_lidar']
        ),

        #Node(
        #    package='status_to_voice',
        #    executable='status_to_voice',
        #    name='status_to_voice',
        #),
        

        Node(
            package='cmd_vel_float_changer',
            executable='cmd_vel_float_changer',
            name='cmd_vel_float_changer',
        ),

        # include launch files set the above
        included_launch1,
        included_launch2,
        included_launch3,
        included_launch4
    ])
