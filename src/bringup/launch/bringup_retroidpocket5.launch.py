import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # include some other launch files
    # launch file for turtlebot3_house
    bringup_dir = get_package_share_directory('bringup')   # package name having a launch file
    included_launch1 = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                    bringup_dir + '/launch/bringup_rp5.launch.py'))

    # launch file for cartographer
    zed_dir = get_package_share_directory('bringup')
    included_launch2 = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                    zed_dir + '/launch/cartographer_imu.launch.py'), 
    )

    nav_dir = get_package_share_directory('nav2_bringup')
    included_launch3 = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                    nav_dir + '/launch/my_navigation.launch.py'), 
    )

    
    return LaunchDescription([

        launch.actions.LogInfo(
            msg="Start zed2i,cartographer,navigation"
        ),


        # include launch files set the above
        included_launch1,
        included_launch2,
        included_launch3

    ])