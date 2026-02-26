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
                'max_linear': 0.20,
                'max_angular': 1.2,
                'wheelbase': 0.23,
                'timeout_sec': 0.25,
                'mps_at_100pct': 0.60,
                'max_motor_pct': 35,
                'min_motor_pct': 18,
            }]
        )
    ])