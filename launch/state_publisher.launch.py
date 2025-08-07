import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Path to your robot xacro file
    xacro_file = PathJoinSubstitution([
        FindPackageShare('wall_follower_bot'),
        'models/urdf',
        'wall_bot.xacro'
    ])

    # Robot description (processed xacro)
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Absolute path to your maze world file
    maze_world_path = PathJoinSubstitution([
        FindPackageShare('wall_follower_bot'),
        'worlds',
        'big_maze.world'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation clock if true'),

        # Launch Gazebo with your maze world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', maze_world_path,
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Start robot_state_publisher node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
        ),

        # Spawn robot entity in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'wall_bot', '-topic', 'robot_description'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # Launch wall follower node
        Node(
            package='wall_follower_bot',
            executable='wall_follower_bot_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
