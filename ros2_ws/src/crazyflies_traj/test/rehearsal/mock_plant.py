#!/usr/bin/env python3
"""
TEST-ONLY mock of the crazyflies-gs plant for the offline rehearsal. Never run
it on a lab domain: run_rehearsal.sh refuses ROS_DOMAIN_ID 0 and 66.

It impersonates the parts of the plant graph traj_player reads and writes:

* node name ``cmd_gate_<ns>`` (the plant's per-vehicle command gate, traj_player's
  server_node check; a one-vehicle plant names it ``cmd_gate``), subscribing
  ``/<ns>/cmd_full_state`` and ``/<ns>/cmd_position`` with the server's QoS
  (rclpy default: reliable, volatile, keep last 10);
* ``/<ns>/odom`` (nav_msgs/Odometry, 100 Hz), ``/poses`` (NamedPoseArray, 100 Hz,
  best effort as on the rig), ``/<ns>/status`` (Status, ``--status-hz``),
  ``/<ns>/pad_z0`` (latched PointStamped).

Script (times from mock start): rest on the pad at z0 with supervisor bits
IS_ARMED|AUTO_ARM|CAN_FLY (14) until ``--takeoff-at``, climb to z0 + hover in
1 s with IS_FLYING set (30), hover. Once commands arrive the vehicle follows
the setpoint with a first-order lag (``--tau``). Like the firmware watchdog
(supervisor.c COMMANDER_WDT_TIMEOUT_SHUTDOWN), 2 s without a command after
streaming began sets IS_LOCKED and drops the vehicle to the pad.

Fault injection, in seconds after the FIRST command received:
``--lock-at`` (IS_LOCKED), ``--tumble-at``, ``--odom-stop-at`` (odom silent),
``--drift-at`` (the vehicle stops following and slides away: tracking error).

Every received command, every status transition and the maximum number of
publishers seen on the command topics is written as JSON lines to
``--record``; check_recording.py judges it.
"""

import argparse
import json
import math
import time

from crazyflie_interfaces.msg import FullState, Position, Status
from geometry_msgs.msg import PointStamped
from motion_capture_tracking_interfaces.msg import NamedPose, NamedPoseArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)

GROUND = 2 | 4 | 8
FLYING = GROUND | 16
TUMBLED, LOCKED = 32, 64


class MockPlant(Node):

    def __init__(self, a):
        super().__init__(f'cmd_gate_{a.ns}')
        self.a = a
        self.t0 = time.monotonic()
        # O_APPEND: intruder.py and run_rehearsal.sh append marker lines to the
        # same file; a plain 'w' handle would overwrite them at its own offset.
        open(a.record, 'w').close()
        self.rec = open(a.record, 'a', buffering=1)
        self.pos = [a.x0, a.y0, a.z0]
        self.vel = [0.0, 0.0, 0.0]
        self.bits = GROUND
        self.cmd = None
        self.t_cmd = None
        self.t_first_cmd = None
        self.odom_on = True
        self.following = True
        self.drift_v = (0.3, 0.0, 0.0)
        self.max_pubs = 0
        self.airborne_logged = False

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST)
        ns = a.ns
        self.odom_pub = self.create_publisher(Odometry, f'/{ns}/odom', 10)
        self.poses_pub = self.create_publisher(NamedPoseArray, '/poses', best_effort)
        self.status_pub = self.create_publisher(Status, f'/{ns}/status', 10)
        pz = self.create_publisher(PointStamped, f'/{ns}/pad_z0', latched)
        m = PointStamped()
        m.header.frame_id = 'mocap'
        m.point.x, m.point.y, m.point.z = a.x0, a.y0, a.z0
        pz.publish(m)
        self.create_subscription(FullState, f'/{ns}/cmd_full_state', self.on_full_state, 10)
        self.create_subscription(Position, f'/{ns}/cmd_position', self.on_position, 10)
        self.cmd_topics = [f'/{ns}/{s}' for s in ('cmd_full_state', 'cmd_position',
                                                  'cmd_vel_legacy', 'cmd_velocity_world',
                                                  'cmd_hover')]
        self.create_timer(0.01, self.step)
        self.create_timer(1.0 / a.status_hz, self.pub_status)
        self.log('start', z0=a.z0, takeoff_at=a.takeoff_at)

    def now(self):
        return time.monotonic() - self.t0

    def log(self, ev, **kw):
        kw.update(ev=ev, t=round(self.now(), 4), mono=round(time.monotonic(), 4))
        self.rec.write(json.dumps(kw) + '\n')

    def on_full_state(self, msg):
        p = msg.pose.position
        v = msg.twist.linear
        self._cmd('full_state', (p.x, p.y, p.z), (v.x, v.y, v.z))

    def on_position(self, msg):
        self._cmd('position', (msg.x, msg.y, msg.z), (0.0, 0.0, 0.0))

    def _cmd(self, kind, pos, vel):
        now = self.now()
        if self.t_first_cmd is None:
            self.t_first_cmd = now
        self.t_cmd = now
        self.cmd = pos
        self.log('cmd', kind=kind, pos=[round(c, 5) for c in pos],
                 vel=[round(c, 5) for c in vel], bits=self.bits,
                 at=[round(c, 5) for c in self.pos])

    def since_first(self, t):
        return t is not None and self.t_first_cmd is not None and \
            self.now() - self.t_first_cmd >= t

    def step(self):
        a, now, dt = self.a, self.now(), 0.01
        prev = list(self.pos)
        # publisher census on the command topics (the dry run must never add one)
        n = sum(self.count_publishers(t) for t in self.cmd_topics)
        if n > self.max_pubs:
            self.max_pubs = n
            self.log('publishers', n=n)
        # scripted takeoff (the HL takeoff the operator does before the player)
        if self.bits & LOCKED:
            self.pos[2] = max(a.z0, self.pos[2] - 1.0 * dt)
        elif now >= a.takeoff_at and self.t_first_cmd is None:
            self.bits = FLYING
            frac = min(1.0, (now - a.takeoff_at) / 1.0)
            self.pos[2] = a.z0 + a.hover * frac
            if frac >= 1.0 and not self.airborne_logged:
                self.airborne_logged = True
                self.log('airborne', pos=list(self.pos))
        # following the commands
        if self.cmd is not None and not self.bits & LOCKED:
            if self.since_first(a.drift_at):
                if self.following:
                    self.following = False
                    self.log('fault', what='drift')
                self.pos = [self.pos[i] + self.drift_v[i] * dt for i in range(3)]
            else:
                k = min(1.0, dt / a.tau)
                self.pos = [self.pos[i] + k * (self.cmd[i] - self.pos[i]) for i in range(3)]
            if now - self.t_cmd > 2.0:
                self.bits |= LOCKED
                self.bits &= ~16
                self.log('watchdog_lock', silent_s=round(now - self.t_cmd, 3))
        if self.since_first(a.lock_at) and not self.bits & LOCKED:
            self.bits = (self.bits | LOCKED) & ~16
            self.log('fault', what='lock')
        if self.since_first(a.tumble_at) and not self.bits & TUMBLED:
            self.bits |= TUMBLED
            self.log('fault', what='tumble')
        if self.since_first(a.odom_stop_at) and self.odom_on:
            self.odom_on = False
            self.log('fault', what='odom_stop')
        # world-frame velocity by finite difference (the real hw twist is
        # body-frame; the player only uses its norm, which is frame-invariant)
        self.vel = [(self.pos[i] - prev[i]) / dt for i in range(3)]
        self.pub_state()
        if a.duration and now > a.duration:
            self.log('end', max_publishers=self.max_pubs)
            raise SystemExit(0)

    def pub_state(self):
        stamp = self.get_clock().now().to_msg()
        if self.odom_on:
            o = Odometry()
            o.header.stamp = stamp
            o.header.frame_id = 'world'
            o.child_frame_id = self.a.ns
            o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = self.pos
            o.pose.pose.orientation.w = 1.0
            o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = self.vel
            self.odom_pub.publish(o)
        pa = NamedPoseArray()
        pa.header.stamp = stamp
        pa.header.frame_id = 'mocap'
        npose = NamedPose()
        npose.name = self.a.ns
        npose.pose.position.x, npose.pose.position.y, npose.pose.position.z = self.pos
        npose.pose.orientation.w = 1.0
        pa.poses.append(npose)
        self.poses_pub.publish(pa)

    def pub_status(self):
        s = Status()
        s.header.stamp = self.get_clock().now().to_msg()
        s.supervisor_info = self.bits
        s.battery_voltage = 4.0
        self.status_pub.publish(s)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ns', default='cf1')
    ap.add_argument('--record', required=True)
    ap.add_argument('--x0', type=float, default=0.0)
    ap.add_argument('--y0', type=float, default=0.0)
    ap.add_argument('--z0', type=float, default=0.05)
    ap.add_argument('--hover', type=float, default=0.5)
    ap.add_argument('--takeoff-at', type=float, default=2.0)
    ap.add_argument('--tau', type=float, default=0.1)
    ap.add_argument('--status-hz', type=float, default=5.0)
    ap.add_argument('--lock-at', type=float, default=None)
    ap.add_argument('--tumble-at', type=float, default=None)
    ap.add_argument('--odom-stop-at', type=float, default=None)
    ap.add_argument('--drift-at', type=float, default=None)
    ap.add_argument('--duration', type=float, default=60.0)
    a = ap.parse_args()
    if a.takeoff_at < 0:
        a.takeoff_at = math.inf                 # never takes off
    rclpy.init()
    node = MockPlant(a)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.log('end', max_publishers=node.max_pubs)
        node.rec.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
