from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='mecanum_micro_ros_agent',
            output="screen",
            #arguments=["udp4", "-p", "8888", "-v6"]
            arguments=["serial", "--dev", "/dev/ttyACM0"]
            #arguments=["serial","--dev", "/dev/serial/by-path/pci-0000:00:14.0-usb-0:3.3:1.0-port0"]
            #serial --dev /dev/ttyACM0
        )


        
    ])
