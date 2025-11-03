import launch
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
        # Declare launch arguments
        use_sim_time = LaunchConfiguration('use_sim_time', default='false')

        bringup_dir = get_package_share_directory('bringup')   # package name having a launch file
        included_launch1 = launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                        bringup_dir + '/launch/bringup_jetson_launch.py'))
        
        included_launch2 = launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                        bringup_dir + '/launch/cartographer_imu.launch.py'))

        nav_dir = get_package_share_directory('nav2_bringup')
        included_launch3 = launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                        nav_dir + '/launch/navigation_launch.py'),
        )

        # ZED Goal Publisher Launch
        zed_goal_publisher_dir = get_package_share_directory('zed_goal_publisher')
        zed_goal_publisher_launch = launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                        zed_goal_publisher_dir + '/launch/zed_goal_publisher.launch.py'),
        )

        return LaunchDescription([

        launch.actions.LogInfo(
                msg="Start zed2i,cartographer,navigation"
        ),

        zed_goal_publisher_launch,

        # include launch files set the above
        included_launch1,
        included_launch2,
        #emcl2_launch,
        included_launch3

])