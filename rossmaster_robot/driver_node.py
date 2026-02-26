import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from .Rosmaster_Lib import Rosmaster


class RossmasterDriver(Node):
    def __init__(self):
        super().__init__('rossmaster_driver')

        # Serial
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)

        # cmd_vel clamp
        self.declare_parameter('max_linear', 0.20)    # m/s clamp for incoming cmd_vel
        self.declare_parameter('max_angular', 1.2)    # rad/s clamp

        # robot geometry
        self.declare_parameter('wheelbase', 0.23)     # meters

        # safety
        self.declare_parameter('timeout_sec', 0.25)

        # motor mapping
        self.declare_parameter('mps_at_100pct', 0.60)  # guessed speed at 100% motor
        self.declare_parameter('max_motor_pct', 35)    # hard cap to prevent bolting
        self.declare_parameter('min_motor_pct', 18)    # deadband compensation

        port = str(self.get_parameter('serial_port').value)
        baud = int(self.get_parameter('baudrate').value)

        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)

        self.mps_at_100pct = float(self.get_parameter('mps_at_100pct').value)
        self.max_motor_pct = int(self.get_parameter('max_motor_pct').value)
        self.min_motor_pct = int(self.get_parameter('min_motor_pct').value)

        self.get_logger().info(f"Initializing Rosmaster on {port} @ {baud}...")
        self.robot = Rosmaster(com=port, baudrate=baud)
        self.get_logger().info("Rosmaster initialized successfully.")

        self.last_cmd_time = time.time()
        self.last_cmd = Twist()
        self.last_debug_time = 0.0

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_timer(0.05, self.watchdog_timer)  # 20 Hz

        self.get_logger().info(
            f"Params: max_linear={self.max_linear}, max_angular={self.max_angular}, "
            f"wheelbase={self.wheelbase}, timeout_sec={self.timeout_sec}, "
            f"mps_at_100pct={self.mps_at_100pct}, max_motor_pct={self.max_motor_pct}, "
            f"min_motor_pct={self.min_motor_pct}"
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

        def to_pct(mps: float) -> int:
            if self.mps_at_100pct <= 1e-6:
                return 0

            pct = int((mps / self.mps_at_100pct) * 100.0)

            # cap
            if pct > self.max_motor_pct:
                pct = self.max_motor_pct
            elif pct < -self.max_motor_pct:
                pct = -self.max_motor_pct

            # deadband compensation
            if pct != 0 and abs(pct) < self.min_motor_pct:
                pct = self.min_motor_pct if pct > 0 else -self.min_motor_pct

            return pct

        l = to_pct(v_l)
        r = to_pct(v_r)

        # Debug once per second so we can see what it's commanding
        now = time.time()
        if now - self.last_debug_time > 1.0:
            self.last_debug_time = now
            self.get_logger().info(f"cmd_vel v={v:.3f} w={w:.3f} -> motor L={l} R={r}")

        try:
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