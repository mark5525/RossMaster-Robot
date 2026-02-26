import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .Rosmaster_Lib import Rosmaster


class RossmasterDriver(Node):
    def __init__(self):
        super().__init__('rossmaster_driver')

        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)

        # tuning params
        self.declare_parameter('max_linear', 0.4)     # m/s
        self.declare_parameter('max_angular', 1.5)    # rad/s
        self.declare_parameter('wheelbase', 0.23)     # meters (adjust)
        self.declare_parameter('timeout_sec', 0.5)    # stop if no cmd_vel

        port = str(self.get_parameter('serial_port').value)
        baud = int(self.get_parameter('baudrate').value)

        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)

        self.get_logger().info(f"Initializing Rosmaster on {port} @ {baud}...")
        self.robot = Rosmaster(com=port, baudrate=baud)
        self.get_logger().info("Rosmaster initialized successfully.")

        self.last_cmd_time = time.time()
        self.last_cmd = Twist()

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_timer(0.05, self.watchdog_timer)  # 20 Hz watchdog

    def cmd_vel_cb(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = time.time()

    def watchdog_timer(self):
        if (time.time() - self.last_cmd_time) > self.timeout_sec:
            # stop
            self.send_stop()
            return
        self.send_twist(self.last_cmd)

    def send_stop(self):
        try:
            # adjust if your API differs
            self.robot.set_motor(0, 0, 0, 0)
        except Exception as e:
            self.get_logger().error(f"Stop failed: {e}")

    def send_twist(self, msg: Twist):
        # clamp
        v = max(min(msg.linear.x, self.max_linear), -self.max_linear)
        w = max(min(msg.angular.z, self.max_angular), -self.max_angular)

        # differential drive model: v_left/right = v ± w*wheelbase/2
        v_l = v - (w * self.wheelbase / 2.0)
        v_r = v + (w * self.wheelbase / 2.0)

        # scale to motor percent [-100, 100] using max_linear as reference
        def to_pct(x):
            if self.max_linear <= 1e-6:
                return 0
            return int(max(min(x / self.max_linear, 1.0), -1.0) * 100)

        l = to_pct(v_l)
        r = to_pct(v_r)

        try:
            self.robot.set_motor(l, l, r, r)
        except Exception as e:
            self.get_logger().error(f"Motor command failed: {e}")


def main():
    rclpy.init()
    node = RossmasterDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()