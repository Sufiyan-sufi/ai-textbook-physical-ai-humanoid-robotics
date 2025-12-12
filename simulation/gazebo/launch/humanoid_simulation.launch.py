from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import TextSubstitution


def generate_launch_description():
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_room.world',
        description='Choose one of the world files from `/simulation/gazebo/worlds`'
    )

    # Get the path to the world file
    world_path = PathJoinSubstitution([
        FindPackageShare('humanoid_description'),
        'worlds',
        LaunchConfiguration('world')
    ])

    # Launch Gazebo server
    gzserver = Node(
        package='gazebo_ros',
        executable='gzserver',
        arguments=[world_path],
        output='screen'
    )

    # Launch Gazebo client
    gzclient = Node(
        package='gazebo_ros',
        executable='gzclient',
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command([
                PathJoinSubstitution([
                    FindPackageShare('humanoid_description'),
                    'urdf',
                    'humanoid.urdf.xacro'
                ])
            ])
        }]
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Spawn entity node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'humanoid_robot',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '1.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gzserver,
        gzclient,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity
    ])