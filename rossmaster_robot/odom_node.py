#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time as RosTime

from .Rosmaster_Lib import Rosmaster


def quat_from_yaw(yaw: float):
    """Return (x,y,z,w) quaternion for yaw-only rotation."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class RossmasterOdom(Node):
    def __init__(self):
        super().__init__('rossmaster_odom')

        # Serial
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)

        # Robot geometry / encoder params
        self.declare_parameter('wheel_radius', 0.0325)    # meters (example)
        self.declare_parameter('wheelbase', 0.23)         # meters (distance between left/right wheel tracks)
        self.declare_parameter('ticks_per_rev', 1560)     # example — you may need to adjust
        self.declare_parameter('use_4wd_average', True)   # X3 has 4 motors; average left pair & right pair

        # Frames
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # Publish rate
        self.declare_parameter('rate_hz', 30.0)

        port = str(self.get_parameter('serial_port').value)
        baud = int(self.get_parameter('baudrate').value)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        self.use_4wd_average = bool(self.get_parameter('use_4wd_average').value)

        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        rate_hz = float(self.get_parameter('rate_hz').value)
        self.dt_target = 1.0 / max(rate_hz, 1.0)

        self.get_logger().info(f"Opening Rosmaster on {port}@{baud} for encoder odom...")
        self.robot = Rosmaster(com=port, baudrate=baud)

        # IMPORTANT: start background receive + auto report
        self.robot.create_receive_threading()
        # enable automatic reports (encoders update via receive thread)
        self.robot.set_auto_report_state(True, forever=False)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Pose state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Encoder state
        self.prev_time = time.time()
        self.prev_enc = self.read_encoders()

        self.create_timer(self.dt_target, self.update)

        self.get_logger().info(
            f"Odom running: wheel_radius={self.wheel_radius}m wheelbase={self.wheelbase}m "
            f"ticks_per_rev={self.ticks_per_rev} frames: {self.odom_frame}->{self.base_frame}"
        )

    def read_encoders(self):
        m1, m2, m3, m4 = self.robot.get_motor_encoder()
        # For X3, typically left side = (m1,m2), right side = (m3,m4) OR similar.
        # We won’t assume wiring; we just average pairs in order:
        return (m1, m2, m3, m4)

    def ticks_to_meters(self, ticks: float) -> float:
        # meters per tick = (2πR) / ticks_per_rev
        return (2.0 * math.pi * self.wheel_radius) * (ticks / self.ticks_per_rev)

    def update(self):
        now = time.time()
        dt = now - self.prev_time
        if dt <= 0.0:
            return

        enc = self.read_encoders()
        d = [enc[i] - self.prev_enc[i] for i in range(4)]
        self.prev_enc = enc
        self.prev_time = now

        # Convert ticks -> distance per wheel
        ds = [self.ticks_to_meters(di) for di in d]

        if self.use_4wd_average:
            # average left pair and right pair (assumes motors 1&2 left, 3&4 right)
            dl = 0.5 * (ds[0] + ds[1])
            dr = 0.5 * (ds[2] + ds[3])
        else:
            # fallback: treat m1 as left, m3 as right
            dl = ds[0]
            dr = ds[2]

        # Differential-drive kinematics
        d_center = 0.5 * (dr + dl)
        d_yaw = (dr - dl) / self.wheelbase

        # Integrate pose
        yaw_mid = self.yaw + 0.5 * d_yaw
        self.x += d_center * math.cos(yaw_mid)
        self.y += d_center * math.sin(yaw_mid)
        self.yaw = (self.yaw + d_yaw + math.pi) % (2.0 * math.pi) - math.pi

        # Velocities
        v = d_center / dt
        w = d_yaw / dt

        # Publish odom + TF
        stamp = self.get_clock().now().to_msg()

        self.publish_tf(stamp)
        self.publish_odom(stamp, v, w)

    def publish_tf(self, stamp: RosTime):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0
        qx, qy, qz, qw = quat_from_yaw(self.yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def publish_odom(self, stamp: RosTime, v: float, w: float):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        msg.pose.pose.position.x = float(self.x)
        msg.pose.pose.position.y = float(self.y)
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = quat_from_yaw(self.yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = float(v)
        msg.twist.twist.angular.z = float(w)

        self.odom_pub.publish(msg)


def main():
    rclpy.init()
    node = RossmasterOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()