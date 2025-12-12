from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tutorials',
            executable='simple_publisher',
            name='simple_publisher',
            output='screen'
        ),
        Node(
            package='tutorials',
            executable='simple_subscriber',
            name='simple_subscriber',
            output='screen'
        )
    ])