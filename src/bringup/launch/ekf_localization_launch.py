import launch
import launch_ros.actions

def generate_launch_description():
    ekf_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            '//home/orinnx/ros2_ws/src/bringup/config/ekf_fusion.yaml'
        ],
        remappings=[
            ('/odometry/filtered', '/fusion_odom')
        ]
    )

    return launch.LaunchDescription([
        ekf_node
    ])