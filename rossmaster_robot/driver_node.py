import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from .Rosmaster_Lib import Rosmaster


class RossmasterDriver(Node):
    def __init__(self):
        super().__init__('rossmaster_driver')

        # Serial params
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)

        # cmd_vel limits (these clamp incoming Twist)
        self.declare_parameter('max_linear', 0.15)     # m/s
        self.declare_parameter('max_angular', 1.0)    # rad/s

        # Robot geometry
        self.declare_parameter('wheelbase', 0.23)     # meters (tune if turning radius feels off)

        # Safety: stop if cmd_vel stops arriving
        self.declare_parameter('timeout_sec', 0.25)   # seconds

        # Motor scaling (THIS is what prevents bolting)
        # mps_at_100pct = robot forward speed (m/s) when motors commanded at 100
        self.declare_parameter('mps_at_100pct', 0.6)  # start guess; tune later
        # Hard cap on motor percent output regardless of cmd_vel
        self.declare_parameter('max_motor_pct', 20)   # 10-30 is a good range indoors

        port = str(self.get_parameter('serial_port').value)
        baud = int(self.get_parameter('baudrate').value)

        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)

        self.mps_at_100pct = float(self.get_parameter('mps_at_100pct').value)
        self.max_motor_pct = int(self.get_parameter('max_motor_pct').value)

        self.get_logger().info(f"Initializing Rosmaster on {port} @ {baud}...")
        self.robot = Rosmaster(com=port, baudrate=baud)
        self.get_logger().info("Rosmaster initialized successfully.")

        self.last_cmd_time = time.time()
        self.last_cmd = Twist()

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_timer(0.05, self.watchdog_timer)  # 20 Hz

        self.get_logger().info(
            f"Params: max_linear={self.max_linear} m/s, max_angular={self.max_angular} rad/s, "
            f"wheelbase={self.wheelbase} m, timeout={self.timeout_sec} s, "
            f"mps_at_100pct={self.mps_at_100pct}, max_motor_pct={self.max_motor_pct}"
        )

    def cmd_vel_cb(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = time.time()

    def watchdog_timer(self):
        if (time.time() - self.last_cmd_time) > self.timeout_sec:
            self.send_stop()
            return
        self.send_twist(self.last_cmd)

    def send_stop(self):
        try:
            self.robot.set_motor(0, 0, 0, 0)
        except Exception as e:
            self.get_logger().error(f"Stop failed: {e}")

    def send_twist(self, msg: Twist):
        # Clamp incoming cmd_vel
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        if v > self.max_linear:
            v = self.max_linear
        elif v < -self.max_linear:
            v = -self.max_linear

        if w > self.max_angular:
            w = self.max_angular
        elif w < -self.max_angular:
            w = -self.max_angular

        # Differential drive wheel speeds (m/s)
        v_l = v - (w * self.wheelbase / 2.0)
        v_r = v + (w * self.wheelbase / 2.0)

        # Convert m/s -> motor percent with a HARD cap to prevent bolting
        def to_pct(mps: float) -> int:
            if self.mps_at_100pct <= 1e-6:
                return 0
            pct = int((mps / self.mps_at_100pct) * 100.0)

            if pct > self.max_motor_pct:
                pct = self.max_motor_pct
            elif pct < -self.max_motor_pct:
                pct = -self.max_motor_pct

            return pct

        l = to_pct(v_l)
        r = to_pct(v_r)

        try:
            # Yahboom uses 4 motors; keep left side same, right side same
            self.robot.set_motor(l, l, r, r)
        except Exception as e:
            self.get_logger().error(f"Motor command failed: {e}")


def main():
    rclpy.init()
    node = RossmasterDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()