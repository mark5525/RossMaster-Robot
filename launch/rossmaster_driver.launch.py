from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rossmaster_robot',
            executable='driver_node',
            name='rossmaster_driver',
            output='screen',
        )
    ])
