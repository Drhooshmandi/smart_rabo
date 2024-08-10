#!/usr/bin/env python3

from os.path import join
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import PythonExpression

from launch_ros.actions import Node


def generate_launch_description():
    # Get smart_rabo package's share directory path
    bcr_bot_path = get_package_share_directory('smart_rabo')
    
    # Path to the Xacro file
    xacro_path = join(bcr_bot_path, 'urdf', 'robot.urdf.xacro')
    
    robot_namespace = LaunchConfiguration("robot_namespace", default='')

    # Launch the robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_path])
        }],
        remappings=[
            ('/joint_states', PythonExpression(['"', robot_namespace, '/joint_states"'])),
        ]
    )

    # Launch the spawn_entity node to spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', "/robot_description",
            '-entity', PythonExpression(['"', robot_namespace, '_robot"']),  # default entity name
            '-z', "0.28"
        ]
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument("robot_namespace", default_value=robot_namespace),
        robot_state_publisher,
        spawn_entity
    ])

