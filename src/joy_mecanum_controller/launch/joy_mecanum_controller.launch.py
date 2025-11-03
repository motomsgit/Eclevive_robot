from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux',
            output='screen',
            parameters=[
                {'dev': '/dev/input/js0',
                 'dev_name': 'DualSense Wireless Controller',
                 'deadzone': 0.05,
                 'autorepeat_rate': 20.0,
                }
            ]
        ),

        Node(
            package='joy_mecanum_controller',
            executable='joy_mecanum_controller_node',
            name='joy_mecanum_controller',
            output='screen',
            
        )
    ])
