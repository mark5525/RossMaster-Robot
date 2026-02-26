import rclpy
from rclpy.node import Node

from .Rosmaster_Lib import Rosmaster


class RossmasterDriver(Node):

    def __init__(self):
        super().__init__('rossmaster_driver')

        self.get_logger().info("Initializing Rosmaster...")
        self.robot = Rosmaster()
        self.get_logger().info("Rosmaster initialized successfully.")


def main():
    rclpy.init()
    node = RossmasterDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
