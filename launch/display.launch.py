import os
from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    # Get the package share directories
    robo_description_share_dir = get_package_share_directory('smart_rabo')
   # micro_ROS_Agent_share_dir = get_package_share_directory('micro-ROS-Agent')
    gazebo_ros_share_dir = get_package_share_directory('gazebo_ros')
    

   # world_file = LaunchConfiguration("world_file", default = join(robo_description_share_dir, 'worlds', 'small_warehouse.sdf'))

    # Declare the launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'urdf_path',
            default_value=os.path.join(robo_description_share_dir, 'urdf', 'robot.urdf.xacro'),
            description='Path to the URDF file'
        ),
        DeclareLaunchArgument(
            'rviz_config_path',
            default_value=os.path.join(robo_description_share_dir , 'rviz', 'urdf_config.rviz'),
            description='Path to the RViz config file'
        ),
      
    ]

    # Launch the robot_state_publisher node
    robot_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )


  # Launch the robot_state_publisher node
    joint_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', LaunchConfiguration('urdf_path')])
        }]
    )

  

  

    # Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_path')]
    )

    # Create the launch description and populate
    ld = LaunchDescription(declared_arguments + [
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])

    return ld
