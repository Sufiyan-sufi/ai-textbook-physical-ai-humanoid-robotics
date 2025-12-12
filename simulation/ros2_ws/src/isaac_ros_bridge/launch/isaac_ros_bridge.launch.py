from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('isaac_ros_bridge'),
            'config',
            'isaac_ros_config.yaml'
        ]),
        description='Path to the Isaac ROS configuration file'
    )

    # Isaac ROS bridge node
    isaac_ros_bridge_node = Node(
        package='isaac_ros_bridge',
        executable='isaac_ros_bridge_node',
        name='isaac_ros_bridge',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # ROS bridge server for Isaac Sim
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[
            {'use_sim_time': True},
            {'port': 9090}
        ],
        output='screen'
    )

    # Robot state publisher (to publish TF and robot state)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': True}
        ],
        output='screen'
    )

    return LaunchDescription([
        config_file_arg,
        isaac_ros_bridge_node,
        rosbridge_server,
        robot_state_publisher
    ])