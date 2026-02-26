import rclpy
from rclpy.node import Node

from .Rosmaster_Lib import Rosmaster


class RossmasterDriver(Node):
    def __init__(self):
        super().__init__('rossmaster_driver')

        # configurable port + baud
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)

        port = str(self.get_parameter('serial_port').value)
        baud = int(self.get_parameter('baudrate').value)

        self.get_logger().info(f"Initializing Rosmaster on {port} @ {baud}...")
        self.robot = Rosmaster(com=port, baudrate=baud)
        self.get_logger().info("Rosmaster initialized successfully.")


def main():
    rclpy.init()
    node = RossmasterDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()