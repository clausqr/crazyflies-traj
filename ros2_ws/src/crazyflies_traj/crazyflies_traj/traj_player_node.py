#!/usr/bin/env python3
"""traj_player: read a CSV trajectory, interpolate it, publish FullState setpoints.

Starts once the plant subscribes; holds the last point for hold_s, then stops.
"""
import math
import sys

from crazyflie_interfaces.msg import FullState
from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from crazyflies_traj.trajectory import load_csv


class TrajPlayer(Node):

    def __init__(self):
        super().__init__('traj_player')
        p = self.declare_parameter
        csv_path = p('csv_path', '').value
        ns = p('ns', 'cf1').value
        rate_hz = float(p('rate_hz', 50.0).value)
        yaw = p('yaw', 'fixed').value
        yaw_deg = float(p('yaw_deg', 0.0).value)
        speedup = float(p('speedup', 1.0).value)
        self.hold_s = float(p('hold_s', 3.0).value)
        pad_z0 = float(p('pad_z0', math.nan).value)
        self.done = self.failed = False
        self.t0 = None
        self.z0 = None if math.isnan(pad_z0) else pad_z0
        try:
            self.traj = load_csv(csv_path, speedup=speedup, yaw=yaw, yaw_deg=yaw_deg)
        except ValueError as e:
            self.get_logger().fatal(str(e))
            self.failed = self.done = True
            return
        self.pub = self.create_publisher(FullState, f'/{ns}/cmd_full_state', 10)
        if self.z0 is None:
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(PointStamped, f'/{ns}/pad_z0', self._on_z0, latched)
        self.create_timer(1.0 / rate_hz, self._tick)

    def _on_z0(self, msg):
        self.z0 = msg.point.z

    def _tick(self):
        if self.z0 is None or self.pub.get_subscription_count() == 0:
            return
        now = self.get_clock().now()
        if self.t0 is None:
            self.t0 = now
        t = (now - self.t0).nanoseconds * 1e-9
        if t > self.traj.duration + self.hold_s:
            self.get_logger().info('DONE')
            self.done = True
            return
        self._publish(self.traj.at(min(t, self.traj.duration)))

    def _publish(self, sp):
        m = FullState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'world'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = sp.pos
        m.pose.position.z += self.z0
        m.pose.orientation.w = math.cos(sp.yaw / 2.0)
        m.pose.orientation.z = math.sin(sp.yaw / 2.0)
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = sp.vel
        m.twist.angular.z = 0.0
        m.acc.x, m.acc.y, m.acc.z = sp.acc
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = TrajPlayer()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 1 if node.failed else 0


if __name__ == '__main__':
    sys.exit(main())
