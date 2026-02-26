from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rossmaster_robot',
            executable='driver_node',
            name='rossmaster_driver',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB1',
                'baudrate': 115200,
                'max_linear': 0.4,
                'max_angular': 1.5,
                'wheelbase': 0.23,
                'timeout_sec': 0.5,
            }]
        )
    ])