import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    camera_name_val = 'zed'
    camera_model_val = 'zed2i'
    # include some other launch files
    # launch file for turtlebot3_house
    bringup_dir = get_package_share_directory('bringup')   # package name having a launch file
    included_launch1 = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                    bringup_dir + '/launch/bringup_devices.launch.py'))

    # launch file for zed2i
    zed_dir = get_package_share_directory('zed_wrapper')
    included_launch2 = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                    zed_dir + '/launch/zed_camera.launch.py'), 
                    # some arguments for launch
                    launch_arguments={
                        'camera_name' : 'zed',
                        'camera_model': 'zed2i'  # "arguments(param) name":"value" 
                    }.items(),
    )

    zed_imu = Node(
            package='zed_imu_republisher',
            executable='imu_republisher',
            name='zed_imu_republisher',
        )
    
    #odom_pub = Node(
    #        package='odom_publisher',
    #        executable='odom_publisher',
    #        name='odom_publisher',
    #)
    
    return LaunchDescription([

        launch.actions.LogInfo(
            msg="Start Jetson_devices."
        ),

        
        # include launch files set the above
        included_launch1,
        included_launch2,
        #zed_imu,
        #odom_pub

    ])