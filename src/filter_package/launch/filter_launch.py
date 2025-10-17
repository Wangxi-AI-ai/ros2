from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='filter_package',
            executable='data_publisher',
            name='data_publisher',
            output='screen'
        ),
        Node(
            package='filter_package',
            executable='median_filter',
            name='median_filter',
            output='screen'
        ),
        Node(
            package='filter_package',
            executable='lowpass_filter',
            name='lowpass_filter',
            output='screen'
        )
    ])
