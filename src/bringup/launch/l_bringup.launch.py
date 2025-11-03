import os
import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    #---------------------  RPLiDAR S2 setting -----------------------------------------
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='1000000') #for s2 is 1000000
    #frame_id = LaunchConfiguration('frame_id', default='laser')
    frame_id = LaunchConfiguration('frame_id', default='base_link')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='DenseBoost')
    #---------------------------------------------------------------------------------
   
    channel_type     = launch.substitutions.LaunchConfiguration('channel_type'   , default="serial")
    serial_port      = launch.substitutions.LaunchConfiguration('serial_port'    , default="/dev/ttyUSB0")
    serial_baudrate  = launch.substitutions.LaunchConfiguration('serial_baudrate', default="1000000")
    frame_id         = launch.substitutions.LaunchConfiguration('frame_id'       , default="base_link")
    inverted         = launch.substitutions.LaunchConfiguration('inverted '      , default="false")
    angle_compensate = launch.substitutions.LaunchConfiguration('angle_compensate' , default="true")
    scan_mode        = launch.substitutions.LaunchConfiguration('scan_mode'     , default="DenseBoost")


    argment0 = launch.actions.DeclareLaunchArgument("channel_type ", default_value="1")
    argment1 = launch.actions.DeclareLaunchArgument("serial_port     ", default_value="2")
    argment2 = launch.actions.DeclareLaunchArgument("serial_baudrate", default_value="3")
    argment3 = launch.actions.DeclareLaunchArgument("frame_id", default_value="4")
    argment4 = launch.actions.DeclareLaunchArgument("inverted", default_value="1")
    argment5 = launch.actions.DeclareLaunchArgument("angle_compensate", default_value="2")
    argment6 = launch.actions.DeclareLaunchArgument("scan_mode", default_value="3")
    

    
    return LaunchDescription([
        argment0,
        argment1,
        argment2,
        argment3,
        argment4,
        argment5,
        argment6,
        

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port, 
                         'serial_baudrate': serial_baudrate, 
                         'frame_id': frame_id,
                         'inverted': inverted, 
                         'angle_compensate': angle_compensate,
                           'scan_mode': scan_mode
                         }],
            output='log'),
        
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=["multiserial", "--devs", "/dev/ttyACM0 /dev/ttyACM1"],
            output='screen'
        ),

        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux',
            output='screen',
            parameters=[
                {'dev': '/dev/input/js6',
                 'dev_name': 'DualSense Wireless Controller',
                 'deadzone': 0.03,
                 'autorepeat_rate': 20.0,
                }
            ]
        ),

        Node(
            package='joy_mecanum_controller',
            executable='joy_mecanum_controller_node',
            name='joy_mecanum_controller',
            output='screen',
            
        ),

        Node( 
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='zed2_static_transform',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.25', '0.0', '1.571', 'zed_camera_link', 'base_link'],
    	),

        Node( 
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='lidar_static_transform',
            arguments=['-0.30', '0.0', '0.0', '0.0', '0.0', '0.0', '1.571', 'base_link', 'laser'],
    	)       


    ])
