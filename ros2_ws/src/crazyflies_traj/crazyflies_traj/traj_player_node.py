#!/usr/bin/env python3
"""
traj_player: stream a validated CSV trajectory to the crazyflies-gs plant.

DRY RUN BY DEFAULT. Without ``fly:=true`` the node validates the CSV, prints
the summary and every setpoint it would send, then runs the airborne gates in
shadow against the live graph and logs what it WOULD do. It never creates a
command publisher, so it cannot command anything and never shows up in the
readiness gate D3 publisher count.

With ``fly:=true`` it still publishes NOTHING until the vehicle is airborne and
stable (see ``player_core``), the approach to the first setpoint is planned
inside the limits and no other node publishes on any ``/<ns>/cmd_*`` topic.
Only then does it create its publisher (becoming the single command
authority), wait for the plant's command gate to match it, and stream
``/<ns>/cmd_full_state`` (default) or ``/<ns>/cmd_position`` at ``rate_hz``. A
hover more than ``start_tol`` from the first setpoint is first approached at
``approach_v`` peak (the bounded move of ``plan.approach_plan``), flowing into
the CSV stream without a gap. After the last point it holds for
``hold_s`` and then destroys the publisher: the stream stops, and landing is
the operator's (README "Handing the trajectory to the groundstation": the firmware levels out
after 0.5 s of silence and LOCKS the motors after 2 s, so the land call must
follow at once). It never calls takeoff, land, arm or notify_setpoints_stop.

Any abort (supervisor locked / tumbled / crashed, stale status / odom / poses,
tracking error, another command publisher, SIGINT / SIGTERM, a message on
``/<ns>/traj_abort``) destroys the publisher on the same tick.

Exit codes: 0 DONE (or dry run complete), 1 refused or aborted, 2 bad parameters.
"""

import math
import signal
import sys
import time

from crazyflie_interfaces.msg import FullState, Position, Status
from geometry_msgs.msg import PointStamped
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy)
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Empty

from crazyflies_traj.plan import (approach_plan, bind, check_offline, format_setpoints,
                                  format_summary, make_sampler)
from crazyflies_traj.player_core import Gates, Inputs, Phase, PlayerCore, TERMINAL
from crazyflies_traj.sampler import setpoint_table, YawSpec
from crazyflies_traj.trajectory import Limits, load_csv, norm, TrajError

COMMANDS = {'full_state': 'cmd_full_state', 'position': 'cmd_position'}
# Every low-level command topic the cflib crazyflie_server subscribes per vehicle
# (crazyswarm2 f1e0995 crazyflie_server.py _init_topics_and_services). A
# publisher on ANY of them is a second command path.
CMD_SUFFIXES = ('cmd_full_state', 'cmd_position', 'cmd_vel_legacy', 'cmd_velocity_world',
                'cmd_hover')

# The plant's cmd_gate subscribes cmd_*
# with the rclpy default (reliable, volatile, keep last 10) and relays to the
# server's /<ns>/gated/cmd_* while the daemon's latched /<ns>/cmd_mute is false.
# A best-effort publisher would NOT match a reliable subscriber, so the command
# publisher is reliable too.
CMD_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
# Best-effort subscriptions match both reliable and best-effort publishers
# (/poses is best-effort on the rig; /<ns>/odom and status are
# reliable from the server).
SENSOR_QOS = qos_profile_sensor_data
STATUS_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)
LATCHED_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)


class TrajPlayer(Node):
    """The ROS side of the player: sample the graph, step the core, act."""

    def __init__(self):
        super().__init__('traj_player')
        p = self.declare_parameter
        self.csv_path = p('csv_path', '').value
        self.ns = p('ns', 'cf1').value.strip('/')
        self.rate_hz = float(p('rate_hz', 50.0).value)
        self.frame = p('frame', 'absolute').value
        self.interp = p('interp', 'linear').value
        self.command = p('command', 'full_state').value
        self.acc_ff = bool(p('acc_ff', False).value)
        self.fly = bool(p('fly', False).value)
        self.header = bool(p('header', True).value)
        self.normalize_t = bool(p('normalize_t', False).value)
        self.speedup = float(p('speedup', 1.0).value)
        self.print_setpoints = bool(p('print_setpoints', True).value)
        self.shadow = bool(p('shadow', True).value)
        pad_z0 = float(p('pad_z0', math.nan).value)
        self.poses_topic = p('poses_topic', '/poses').value
        self.world_frame = p('world_frame', 'world').value
        # The plant node that subscribes /<ns>/cmd_*: the per-vehicle gate, named
        # cmd_gate when the plant flies one vehicle and cmd_gate_<ns> for several.
        self.server_node = p('server_node', 'cmd_gate').value
        # The fleet roster (comma-separated namespaces, set by fleet.launch.py):
        # the sync barrier holds at the first setpoint until every present
        # peer is there too. Empty = no barrier (single vehicle).
        self.fleet = [x.strip('/ ') for x in p('fleet', '').value.split(',')
                      if x.strip('/ ') and x.strip('/ ') != self.ns]
        self.peer_ready = set()
        self.limits = Limits(**{k: float(p(k, v.default).value)
                                for k, v in Limits.__dataclass_fields__.items()})
        self.yaw = YawSpec(**{k: p(k, v.default).value
                               for k, v in YawSpec.__dataclass_fields__.items()})
        gate_kw = {}
        for k, v in Gates.__dataclass_fields__.items():
            val = p(k, v.default).value
            gate_kw[k] = bool(val) if isinstance(v.default, bool) else float(val)
        self.gates = Gates(**gate_kw)

        self.exit_code = None
        self.published = 0
        self.pub = None
        self.status = self.odom = self.poses = self.odom_yaw = None
        self.z0 = pad_z0 if math.isfinite(pad_z0) else None
        self.abort_reason = None
        self._last_wait = None
        self._t_start = time.monotonic()
        self._absence_checked = False

        if self.command not in COMMANDS:
            self._fatal(2, f"command must be one of {sorted(COMMANDS)}, got {self.command!r}")
            return
        if not (math.isfinite(self.rate_hz) and 10.0 <= self.rate_hz <= 100.0):
            # >= 10 Hz keeps far inside the 500 ms firmware setpoint watchdog;
            # <= 100 Hz is a rate proven on the same radio link.
            self._fatal(2, f'rate_hz must be in [10, 100], got {self.rate_hz}')
            return
        self.cmd_topic = f'/{self.ns}/{COMMANDS[self.command]}'
        self.cmd_topics = [f'/{self.ns}/{s}' for s in CMD_SUFFIXES]

        try:
            if not self.csv_path:
                raise TrajError('no_csv', 'csv_path parameter is required')
            self.traj = load_csv(self.csv_path, header=self.header, normalize_t=self.normalize_t,
                                 speedup=self.speedup)
            self.yaw_demand = check_offline(self.traj, self.limits, self.frame, self.interp,
                                            self.yaw, self.rate_hz)
            self.core = PlayerCore(self._bind, self.gates, self.limits.start_tol,
                                   shadow=not self.fly, approach_fn=self._approach,
                                   sync=bool(self.fleet))
        except TrajError as exc:
            self._fatal(1, f'REFUSING TO FLY: {exc}')
            return

        self._print_plan()
        log = self.get_logger()
        if not self.fly and not self.shadow:
            log.info('DRY RUN complete (shadow:=false): nothing published, graph not watched')
            self.exit_code = 0
            return
        if self.fly:
            log.warn(f'FLY MODE: will take command authority on {self.cmd_topic} once '
                     f'/{self.ns} is airborne and stable; publishes nothing before that')
        else:
            log.info('DRY RUN: publishes nothing, ever (fly:=true to stream). Shadowing the '
                     'airborne gates against the live graph.')

        self.create_subscription(Status, f'/{self.ns}/status', self._on_status, STATUS_QOS)
        self.create_subscription(Odometry, f'/{self.ns}/odom', self._on_odom, SENSOR_QOS)
        self.create_subscription(NamedPoseArray, self.poses_topic, self._on_poses, SENSOR_QOS)
        self.create_subscription(Empty, f'/{self.ns}/traj_abort', self._on_abort_topic,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        # The ready latch is not a command topic: it is published in the dry run too,
        # so a fleet dry run synchronises like the flight.
        self.ready_pub = self.create_publisher(Empty, f'/{self.ns}/traj_ready', LATCHED_QOS)
        for peer in self.fleet:
            self.create_subscription(Empty, f'/{peer}/traj_ready',
                                     lambda _m, peer=peer: self.peer_ready.add(peer), LATCHED_QOS)
        if self.fleet:
            log.info(f'fleet sync: holding at the first setpoint until {self.fleet} are ready '
                     f'(absent ones skipped; sync_timeout {self.gates.sync_timeout:.0f} s)')
        if self.z0 is None:
            self.create_subscription(PointStamped, f'/{self.ns}/pad_z0', self._on_pad_z0,
                                     LATCHED_QOS)
            log.info(f'WAIT_AIRBORNE: pad z0 from the latched /{self.ns}/pad_z0')
        else:
            log.info(f'pad z0 pinned by parameter: {self.z0:.3f} m')
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)

    # ---- setup helpers ---------------------------------------------------
    def _fatal(self, code, msg):
        self.get_logger().fatal(msg)
        self.exit_code = code

    def _bind(self, anchor_xy, z0):
        return bind(self.traj, self.limits, self.frame, self.interp, z0, anchor_xy, self.yaw,
                    self.rate_hz)

    def _approach(self, pos, first, z0, yaw):
        return approach_plan(pos, first, z0, self.limits, self.gates.approach_v, yaw)

    def _print_plan(self):
        z0 = self.z0
        offset = (0.0, 0.0, z0 if z0 is not None else 0.0)
        z_label = f'world, pad z0 {z0:.3f}' if z0 is not None else 'above pad; z0 not yet known'
        if self.frame == 'relative':
            z_label += '; x, y relative, anchored at trigger'
        path = self.yaw.yaw == 'path'
        yaw_line = (f'path, max rate {self.yaw_demand:.1f} deg/s (yaw_rate_max_dps '
                    f'{self.limits.yaw_rate_max_dps:g}), the approach turns from the odom yaw'
                    if path else f'fixed {self.yaw.yaw_fixed_deg:g} deg')
        lines = format_summary(self.traj, self.interp, self.frame, self.rate_hz, offset, z_label,
                               yaw_line)
        lines.append(f'  command       {self.cmd_topic} (crazyflie_interfaces/'
                     f'{"FullState" if self.command == "full_state" else "Position"}), '
                     f'acc feed-forward {"on" if self.acc_ff else "off"}')
        for line in lines:
            print(line, flush=True)
        if not self.fly and self.print_setpoints:
            sampler = make_sampler(self.traj, self.interp, offset, self.yaw, self.rate_hz,
                                   self.limits)[0]
            table = setpoint_table(sampler, self.rate_hz)
            print(f'DRY RUN setpoints ({z_label}):', flush=True)
            for line in format_setpoints(table, self.acc_ff and self.command == 'full_state',
                                         path):
                print(line, flush=True)

    # ---- subscriptions -----------------------------------------------------
    def _on_status(self, msg):
        self.status = (time.monotonic(), int(msg.supervisor_info))

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        # |v| is frame-invariant: the hw odom twist is body-frame (kalman.stateP*),
        # the sim one world-frame; only the norm is used.
        self.odom = (time.monotonic(), (p.x, p.y, p.z), norm((v.x, v.y, v.z)))
        q = msg.pose.pose.orientation
        self.odom_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _on_poses(self, msg):
        for named in msg.poses:
            if named.name == self.ns:
                q = named.pose.position
                self.poses = (time.monotonic(), (q.x, q.y, q.z))
                return

    def _on_pad_z0(self, msg):
        if self.z0 is None and math.isfinite(msg.point.z):
            self.z0 = float(msg.point.z)
            self.get_logger().info(f'pad z0 {self.z0:.3f} m from /{self.ns}/pad_z0 (latched)')

    def _on_abort_topic(self, _msg):
        self.request_abort(f'/{self.ns}/traj_abort')

    def request_abort(self, why):
        """Flag an abort (signal handler / abort topic); the next tick releases.

        Only sets a flag: safe from a signal handler. The main loop spins at
        least every 50 ms and the timer ticks every 1 / rate_hz, so the
        publisher is gone within one tick.
        """
        if self.abort_reason is None:
            self.abort_reason = why

    # ---- graph -------------------------------------------------------------
    def _foreign_publishers(self):
        me = (self.get_name(), self.get_namespace())
        n = 0
        for topic in self.cmd_topics:
            for info in self.get_publishers_info_by_topic(topic):
                if (info.node_name, info.node_namespace) != me:
                    n += 1
        return n

    def _server_matched(self):
        if self.pub is None:
            return False
        subs = self.get_subscriptions_info_by_topic(self.cmd_topic)
        names = (self.server_node, f'{self.server_node}_{self.ns}')
        server_up = any(s.node_name in names for s in subs)
        return server_up and self.pub.get_subscription_count() >= 1

    def _fleet_waiting(self):
        """Return the present peers (an odom publisher exists) that are not ready yet."""
        return [peer for peer in self.fleet if peer not in self.peer_ready
                and self.count_publishers(f'/{peer}/odom') > 0]

    # ---- the tick ------------------------------------------------------------
    def _tick(self):
        if self.exit_code is not None:
            return
        self._warn_if_absent()
        waiting = self._fleet_waiting() if self.fleet else []
        inp = Inputs(now=time.monotonic(), status=self.status, odom=self.odom,
                     yaw=self.odom_yaw, poses=self.poses, z0=self.z0,
                     foreign_publishers=self._foreign_publishers(),
                     server_matched=self._server_matched(), abort=self.abort_reason,
                     fleet_ready=not waiting, fleet_waiting=', '.join(waiting))
        out = self.core.step(inp)
        log = self.get_logger()
        tag = '' if self.fly else '[DRY RUN] '
        for phase, why in out.entered:
            why = why.replace('/<ns>/', f'/{self.ns}/')
            if phase == Phase.ABORT:
                log.error(f'{tag}ABORT: {why}')
            else:
                log.info(f'{tag}{phase.value}: {why}')
        if out.waiting_on and out.waiting_on.split(' ')[0] != (self._last_wait or '').split(' ')[0]:
            log.info(f'{tag}waiting: ' + out.waiting_on.replace('/<ns>/', f'/{self.ns}/'))
        self._last_wait = out.waiting_on
        if out.ready:
            self.ready_pub.publish(Empty())
            log.info(f'{tag}/{self.ns}/traj_ready latched')
        if out.take_authority and self.fly and self.pub is None:
            msg_type = FullState if self.command == 'full_state' else Position
            self.pub = self.create_publisher(msg_type, self.cmd_topic, CMD_QOS)
            log.warn(f'command authority taken: publisher on {self.cmd_topic}')
        if out.setpoint is not None and self.pub is not None and self.fly:
            self._publish(out.setpoint)
        if out.release:
            self._release()
        if self.core.phase in TERMINAL:
            self._release()
            ok = self.core.phase == Phase.DONE
            if ok:
                log.info(f'{tag}DONE: {self.published} setpoints streamed; publisher destroyed. '
                         'Landing is the operator\'s now.')
            self.exit_code = 0 if ok else 1

    def _warn_if_absent(self):
        """Once, 5 s after start: warn when nothing from this vehicle has arrived.

        A fleet launch names vehicles that may not be on the graph (the deploy
        ladder flies cf1 first, then cf1 + cf2, ...). Their players keep waiting
        up to wait_timeout and exit 1 on their own; the others are unaffected.
        Keyed on received messages, not the publisher census: discovery of a
        five-node fleet took over 2 s on loopback.
        """
        if self._absence_checked or time.monotonic() - self._t_start < 5.0:
            return
        self._absence_checked = True
        if self.status is None and self.odom is None:
            self.get_logger().warn(f'/{self.ns} ABSENT: no /{self.ns}/status or /{self.ns}/odom '
                                   f'message within 5 s (absent, or still connecting); waiting up '
                                   f'to {self.gates.wait_timeout:.0f} s for it, then giving up '
                                   '(other vehicles are unaffected)')

    def _publish(self, sp):
        if self.command == 'full_state':
            m = FullState()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = self.world_frame
            m.pose.position.x, m.pose.position.y, m.pose.position.z = sp.pos
            # Yaw only (roll, pitch 0: the controller derives the tilt from the thrust
            # vector); the server sends angular as rad/s (millirad/s on the radio).
            m.pose.orientation.w = math.cos(sp.yaw / 2.0)
            m.pose.orientation.z = math.sin(sp.yaw / 2.0)
            m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = sp.vel
            m.twist.angular.z = sp.yaw_rate
            if self.acc_ff:
                m.acc.x, m.acc.y, m.acc.z = sp.acc
        else:
            m = Position()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = self.world_frame
            m.x, m.y, m.z = sp.pos
            m.yaw = math.degrees(sp.yaw)          # Position.yaw is degrees (cflib, the firmware)
        self.pub.publish(m)
        self.published += 1

    def _release(self):
        if self.pub is not None:
            self.destroy_publisher(self.pub)
            self.pub = None
            self.get_logger().warn(f'command authority released: {self.cmd_topic} publisher '
                                   f'destroyed after {self.published} setpoints')


def main(args=None):
    """Run until DONE / ABORT / refusal; the exit code is the verdict."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = TrajPlayer()

    def on_signal(signum, _frame):
        node.request_abort(signal.Signals(signum).name)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    try:
        while rclpy.ok() and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.abort_reason is not None and node.exit_code is None:
                node._tick()
    finally:
        node._release()
        code = node.exit_code if node.exit_code is not None else 1
        node.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
