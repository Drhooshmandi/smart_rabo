import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([

        launch.actions.ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'udp4', '--port', '8888', '--dev', '192.168.43.242'],
            output='screen',
        ),
        
         launch.actions.ExecuteProcess(
            cmd=['ros2', 'run', 'smart_rabo', 'controller_node.py'],
            output='screen',
        ),

    ])