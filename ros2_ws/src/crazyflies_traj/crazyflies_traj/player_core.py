r"""
The traj_player state machine, ROS-free (unit-tested on plain data).

    WAIT_AIRBORNE -> ARMED_READY -> APPROACH -> READY -> FLYING -> HOLD -> DONE
          \______________\____________\___________\________\________\____> ABORT

The node samples the graph (status, odom, poses, pad z0, publisher counts) and
the clock into an :class:`Inputs`, calls :meth:`PlayerCore.step`, and turns the
returned :class:`Output` into side effects: create the command publisher
(``take_authority``), publish ``setpoint``, destroy the publisher
(``release``). The core never publishes anything itself and holds no ROS
object, so every gate below is witnessed in unit tests.

"Airborne and stable" (README "Airborne"), ALL of, continuously for
``stable_s``:

* pad z0 known (latched ``/<ns>/pad_z0`` or pinned);
* ``/<ns>/status`` fresh (``status_timeout``) with supervisor bits IS_ARMED (2),
  CAN_FLY (8) and IS_FLYING (16) set, and IS_TUMBLED (32), IS_LOCKED (64),
  IS_CRASHED (128) clear;
* ``/<ns>/odom`` fresh (``odom_timeout``), height ``odom.z - z0`` above ``h_enter``
  (hysteresis: once airborne it only drops out below ``h_exit``), speed at most
  ``v_stable``;
* with ``require_poses``: ``/poses`` for ``<ns>`` fresh and within
  ``anchor_tol`` of ``/<ns>/odom`` (the EKF is anchored to mocap; a dead mocap leaves
  a plausible-looking odom).

APPROACH (``approach_fn(current, first, z0)``, :func:`~crazyflies_traj.plan.approach_plan`)
runs only when the airborne position is more than ``start_tol`` from the first
setpoint: a bounded move to it (peak speed ``approach_v``, ``a_max``, the
geofence), streamed like the trajectory and flowing into FLYING on the same tick
its time is up, so the command stream never pauses (the firmware levels out at
500 ms of silence). Without an ``approach_fn`` a far start is refused
(``start_far``). The move is computed from the odom position at the end of
``stable_s``; the server match may take up to ``authority_timeout``, and
``track_tol`` covers the drift meanwhile (the same exposure as the start check).
With ``yaw: path`` the approach also turns the nose from the measured odom yaw
(``Inputs.yaw``) to the first setpoint's yaw at ``yaw_rate_max_dps``, and it runs
even from a near start when that turn exceeds ``YAW_TOL``: the command never
steps the yaw, so the controller never spins the vehicle to catch up.

READY (``sync=True``, the fleet barrier): once at the first setpoint (after the
approach, or at once) the player holds it, streaming, and reports ready
(``Output.ready``, the node latches ``/<ns>/traj_ready``) until every present
fleet peer is ready too (``Inputs.fleet_ready``, sampled by the node from the
peers' latched ready topics, absent peers left out) or ``sync_timeout`` runs
out, then FLYING on the same tick. Without ``sync`` FLYING follows directly.

Any fatal status bit, stale stream, foreign command publisher, tracking error
above ``track_tol`` or an operator abort ends in ABORT: stop publishing at
once (``release``), never keep streaming.
"""

from dataclasses import dataclass, field
import enum
import math

from crazyflies_traj.sampler import wrap_pi
from crazyflies_traj.trajectory import check_start, norm, TrajError

# crazyflie_interfaces/msg/Status SUPERVISOR_INFO_* (crazyswarm2 f1e0995), plus
# bit 128 which the firmware sets for isCrashed (supervisor.c updateLogData,
# 0x0080) but Status.msg has no constant for.
CAN_BE_ARMED = 1
IS_ARMED = 2
AUTO_ARM = 4
CAN_FLY = 8
IS_FLYING = 16
IS_TUMBLED = 32
IS_LOCKED = 64
IS_CRASHED = 128

REQUIRED_BITS = IS_ARMED | CAN_FLY | IS_FLYING
FATAL_BITS = IS_TUMBLED | IS_LOCKED | IS_CRASHED
_BIT_NAMES = {IS_ARMED: 'IS_ARMED', CAN_FLY: 'CAN_FLY', IS_FLYING: 'IS_FLYING',
              IS_TUMBLED: 'IS_TUMBLED', IS_LOCKED: 'IS_LOCKED', IS_CRASHED: 'IS_CRASHED'}


def bit_names(mask):
    """Return the names of the known bits set in ``mask``, e.g. 'IS_LOCKED|IS_TUMBLED'."""
    return '|'.join(n for b, n in sorted(_BIT_NAMES.items()) if mask & b) or 'none'


class Phase(enum.Enum):
    """Player phases."""

    WAIT_AIRBORNE = 'WAIT_AIRBORNE'
    ARMED_READY = 'ARMED_READY'
    APPROACH = 'APPROACH'
    READY = 'READY'
    FLYING = 'FLYING'
    HOLD = 'HOLD'
    DONE = 'DONE'
    ABORT = 'ABORT'


TERMINAL = (Phase.DONE, Phase.ABORT)
YAW_TOL = math.radians(2.0)   # a takeover yaw this close to the first setpoint's needs no turn


@dataclass(frozen=True)
class Gates:
    """
    Gate thresholds (README "Safety model").

    track_tol is sized to the sample envelope (README "Limits").
    """

    h_enter: float = 0.25        # m above z0 to count as airborne
    h_exit: float = 0.15         # m above z0 below which it no longer does
    v_stable: float = 0.30       # m/s, |v_odom| while settling. The fused-attitude
                                 # EKF velocity bias at rest (0.10 to 0.25 m/s) must
                                 # not block the takeover; the approach peaks at approach_v 0.30
    stable_s: float = 1.0        # s of continuous airborne-and-stable
    status_timeout: float = 3.0  # s; the plant logs status at 1 Hz
    odom_timeout: float = 0.25   # s; the plant logs odom at 100 Hz
    poses_timeout: float = 0.25  # s; mocap streams at about 120 Hz
    require_poses: bool = True
    anchor_tol: float = 0.10     # m, |odom - mocap| while waiting
    track_tol: float = 0.60      # m, |odom - setpoint| in flight
    hold_s: float = 3.0          # s at the last point before DONE
    wait_timeout: float = 120.0  # s in WAIT_AIRBORNE before giving up
    authority_timeout: float = 3.0  # s for the server to match our publisher
    approach_v: float = 0.30     # m/s, peak speed of the approach to the first setpoint
    sync_timeout: float = 30.0   # s in READY waiting for the fleet before going anyway

    def check_sane(self):
        """Refuse inconsistent gates (e.g. no hysteresis band)."""
        vals = [getattr(self, k) for k in self.__dataclass_fields__ if k != 'require_poses']
        if not all(math.isfinite(v) and v > 0.0 for v in vals):
            raise TrajError('bad_gates', 'every gate threshold must be finite and > 0')
        if not self.h_exit < self.h_enter:
            raise TrajError('bad_gates', f'h_exit {self.h_exit} must be below h_enter '
                                         f'{self.h_enter} (hysteresis band)')


@dataclass
class Inputs:
    """One sample of the world. Times are the node's monotonic seconds."""

    now: float
    status: tuple = None        # (t_rx, supervisor_info)
    odom: tuple = None          # (t_rx, (x, y, z), speed)
    yaw: float = None           # odom yaw (rad), the start of the yaw: path turn
    poses: tuple = None         # (t_rx, (x, y, z)) of the rigid body named ns
    z0: float = None
    foreign_publishers: int = 0  # publishers on any /<ns>/cmd_* that are not ours
    server_matched: bool = False  # the plant's cmd_gate subscribes to our command topic
    abort: str = None           # operator abort (SIGINT, abort topic), reason text
    fleet_ready: bool = True    # every present fleet peer is at its start (sync only)
    fleet_waiting: str = ''    # who is not yet, for the log


@dataclass
class Output:
    """What the node must do this tick."""

    phase: Phase
    entered: list = field(default_factory=list)   # [(Phase, reason)] entered this tick
    setpoint: object = None                       # Setpoint to publish, or None
    take_authority: bool = False                  # create the command publisher now
    release: bool = False                         # destroy it now (DONE / ABORT)
    waiting_on: str = None                        # why WAIT_AIRBORNE / READY is still waiting
    ready: bool = False                           # at the first setpoint: latch /<ns>/traj_ready


class PlayerCore:
    """
    The pure state machine. ``bind_fn(anchor_xy, z0)`` returns a bound Plan or
    raises TrajError (geofence at the anchor); ``approach_fn(current, first, z0, yaw)``
    returns the bound Plan of the move to the first setpoint or raises TrajError
    (None: a far start is refused instead); ``shadow`` is the dry run: the same
    gates, but no authority is ever taken and tracking is not checked (nothing
    flies the setpoints, so an error would be meaningless).
    """

    def __init__(self, bind_fn, gates, start_tol, shadow=True, approach_fn=None, sync=False):
        gates.check_sane()
        self.sync = sync
        self.bind_fn = bind_fn
        self.approach_fn = approach_fn
        self.g = gates
        self.start_tol = start_tol
        self.shadow = shadow
        self.phase = Phase.WAIT_AIRBORNE
        self.plan = None
        self.approach = None        # the bound move to the first setpoint, when needed
        self.reason = None
        self.airborne = False       # height hysteresis latch
        self._t_wait0 = None
        self._t_stable0 = None
        self._t_auth0 = None
        self._t_appr0 = None
        self._t_ready0 = None
        self._t_fly0 = None
        self._t_hold0 = None
        self.last_setpoint = None

    # ---- helpers -------------------------------------------------------
    def _enter(self, out, phase, reason):
        self.phase = phase
        out.phase = phase
        out.entered.append((phase, reason))
        if phase in TERMINAL:
            self.reason = reason
            out.release = not self.shadow and self._t_auth0 is not None
            out.setpoint = None

    def _begin(self, out, inp):
        """At the first setpoint: hold for the fleet (sync) or start streaming."""
        if self.sync:
            self._t_ready0 = inp.now
            self._enter(out, Phase.READY, 'at the first setpoint, holding until the fleet is ready')
            out.ready = True
        else:
            self._fly(out, inp, '')

    def _fly(self, out, inp, why):
        self._t_fly0 = inp.now
        self._enter(out, Phase.FLYING, f'streaming {self.plan.traj.duration:.2f} s'
                    + (f' ({why})' if why else ''))

    def _fresh(self, sample, now, timeout):
        return sample is not None and (now - sample[0]) <= timeout

    def _fatal_bits(self, inp):
        if inp.status is not None and inp.status[1] & FATAL_BITS:
            return f'supervisor {bit_names(inp.status[1] & FATAL_BITS)}'
        return None

    def _airborne_now(self, inp):
        """Return None when airborne-and-stable holds this tick, else why not."""
        g, now = self.g, inp.now
        if inp.z0 is None:
            return 'pad z0 unknown (waiting for the latched /<ns>/pad_z0)'
        if not self._fresh(inp.status, now, g.status_timeout):
            return 'no fresh /<ns>/status'
        bits = inp.status[1]
        missing = REQUIRED_BITS & ~bits
        if missing:
            return f'supervisor bits missing: {bit_names(missing)} (have {bits})'
        if not self._fresh(inp.odom, now, g.odom_timeout):
            return 'no fresh odom'
        _, pos, speed = inp.odom
        h = pos[2] - inp.z0
        if self.airborne and h < g.h_exit:
            self.airborne = False
        elif not self.airborne and h >= g.h_enter:
            self.airborne = True
        if not self.airborne:
            return f'height {h:.3f} m above pad (need {g.h_enter:.2f} to enter)'
        if g.require_poses:
            if not self._fresh(inp.poses, now, g.poses_timeout):
                return 'no fresh /poses for the vehicle (mocap)'
            d = norm(tuple(pos[a] - inp.poses[1][a] for a in range(3)))
            if d > g.anchor_tol:
                return f'odom {d:.3f} m from mocap (anchor_tol {g.anchor_tol:.2f}): EKF not anchored'
        if speed > g.v_stable:
            return f'speed {speed:.3f} m/s (need <= {g.v_stable:.2f} to be stable)'
        return None

    def _health(self, inp, sp):
        """Return an abort reason while APPROACH / FLYING / HOLD, or None."""
        g, now = self.g, inp.now
        bad = self._fatal_bits(inp)
        if bad:
            return bad
        if not self._fresh(inp.status, now, g.status_timeout):
            return f'/<ns>/status stale (> {g.status_timeout:.2f} s)'
        if not inp.status[1] & IS_FLYING:
            return 'supervisor IS_FLYING cleared (motors at idle)'
        if not self._fresh(inp.odom, now, g.odom_timeout):
            return f'odom stale (> {g.odom_timeout:.2f} s)'
        if g.require_poses and not self._fresh(inp.poses, now, g.poses_timeout):
            return f'/poses stale (> {g.poses_timeout:.2f} s): mocap lost'
        if inp.foreign_publishers > 0:
            return (f'{inp.foreign_publishers} other publisher(s) on /<ns>/cmd_*: '
                    'single command authority violated')
        if not self.shadow and sp is not None:
            err = norm(tuple(inp.odom[1][a] - sp.pos[a] for a in range(3)))
            if err > g.track_tol:
                return f'tracking error {err:.3f} m > track_tol {g.track_tol:.2f}'
        return None

    # ---- the step ------------------------------------------------------
    def step(self, inp):
        """Advance one tick; return the :class:`Output`."""
        out = Output(phase=self.phase)
        if self.phase in TERMINAL:
            return out
        if inp.abort:
            self._enter(out, Phase.ABORT, f'operator abort: {inp.abort}')
            return out

        if self.phase == Phase.WAIT_AIRBORNE:
            if self._t_wait0 is None:
                self._t_wait0 = inp.now
            bad = self._fatal_bits(inp)
            if bad:
                self._enter(out, Phase.ABORT, f'refusing: {bad} before any command')
                return out
            why = self._airborne_now(inp)
            if why is not None:
                self._t_stable0 = None
                out.waiting_on = why
                if inp.now - self._t_wait0 > self.g.wait_timeout:
                    self._enter(out, Phase.ABORT, f'refusing: not airborne and stable within '
                                                  f'{self.g.wait_timeout:.0f} s ({why})')
                return out
            if self._t_stable0 is None:
                self._t_stable0 = inp.now
            if inp.now - self._t_stable0 < self.g.stable_s:
                out.waiting_on = f'settling ({inp.now - self._t_stable0:.2f} / {self.g.stable_s:.2f} s)'
                return out
            # Airborne and stable: bind the plan to the world and check the start;
            # a far start becomes the approach when there is a way to plan one.
            pos = inp.odom[1]
            try:
                self.plan = self.bind_fn((pos[0], pos[1]), inp.z0)
                d = norm(tuple(self.plan.first[a] - pos[a] for a in range(3)))
                yaw = self.plan.takeover_yaw(inp.yaw)
                turn = abs(wrap_pi(yaw[1] - yaw[0]))
                if (d > self.start_tol or turn > YAW_TOL) and self.approach_fn is not None:
                    self.approach = self.approach_fn(pos, self.plan.first, inp.z0, yaw)
                else:
                    d = check_start(self.plan.first, pos, self.start_tol)
            except TrajError as exc:
                self._enter(out, Phase.ABORT, f'refusing: {exc}')
                return out
            if inp.foreign_publishers > 0:
                self._enter(out, Phase.ABORT, f'refusing: {inp.foreign_publishers} other '
                                              'publisher(s) on /<ns>/cmd_* (single command '
                                              'authority, readiness gate D3)')
                return out
            appr = (f', approach {self.approach.traj.duration:.2f} s'
                    + (f' turning {math.degrees(turn):.1f} deg' if turn > YAW_TOL else '')
                    if self.approach is not None else '')
            self._enter(out, Phase.ARMED_READY, f'airborne and stable at ({pos[0]:.3f}, '
                                                f'{pos[1]:.3f}, {pos[2]:.3f}), z0 {inp.z0:.3f}, '
                                                f'start {d:.3f} m from the first setpoint{appr}')
            self._t_auth0 = inp.now
            out.take_authority = not self.shadow
            return out

        if self.phase == Phase.ARMED_READY:
            bad = self._fatal_bits(inp)
            if bad or inp.foreign_publishers > 0:
                why = bad or f'{inp.foreign_publishers} other publisher(s) on /<ns>/cmd_*'
                self._enter(out, Phase.ABORT, f'refusing: {why}')
                return out
            if not self.shadow and not inp.server_matched:
                if inp.now - self._t_auth0 > self.g.authority_timeout:
                    self._enter(out, Phase.ABORT, 'refusing: the plant gate never matched '
                                                  f'the command publisher within '
                                                  f'{self.g.authority_timeout:.1f} s')
                else:
                    out.waiting_on = 'waiting for the plant gate to match the command topic'
                return out
            if self.approach is not None:
                self._t_appr0 = inp.now
                big_t = self.approach.traj.duration
                d = norm(tuple(self.approach.last[a] - self.approach.first[a] for a in range(3)))
                self._enter(out, Phase.APPROACH, f'approach {d:.3f} m in {big_t:.2f} s, peak '
                                                 f'{1.5 * d / big_t:.2f} m/s')
            else:
                self._begin(out, inp)
            # fall through: the first setpoint goes out on this same tick

        if self.phase == Phase.APPROACH:
            big_t = self.approach.traj.duration
            t = inp.now - self._t_appr0
            sp = self.approach.sampler.at(min(t, big_t))
            why = self._health(inp, sp)
            if why:
                self._enter(out, Phase.ABORT, why)
                return out
            if t >= big_t:
                self._begin(out, inp)
                # fall through: the first CSV setpoint (the same point, zero
                # velocity) goes out on this same tick, so the stream never pauses
            else:
                out.setpoint = sp
                self.last_setpoint = sp
                return out

        if self.phase == Phase.READY:
            sp = self.plan.sampler.at(0.0)
            why = self._health(inp, sp)
            if why:
                self._enter(out, Phase.ABORT, why)
                return out
            waited = inp.now - self._t_ready0
            if inp.fleet_ready:
                self._fly(out, inp, f'fleet ready after {waited:.2f} s')
            elif waited >= self.g.sync_timeout:
                self._fly(out, inp, f'sync_timeout {self.g.sync_timeout:.0f} s: going without {inp.fleet_waiting or "the fleet"}')
            else:
                out.waiting_on = f'fleet: {inp.fleet_waiting or "peers"} not ready yet'
                out.setpoint = sp
                self.last_setpoint = sp
                return out
            # fall through: the first CSV setpoint goes out on this same tick

        if self.phase == Phase.FLYING:
            t = inp.now - self._t_fly0
            sp = self.plan.sampler.at(min(t, self.plan.traj.duration))
            why = self._health(inp, sp)
            if why:
                self._enter(out, Phase.ABORT, why)
                return out
            if t >= self.plan.traj.duration:
                self._t_hold0 = inp.now
                self._enter(out, Phase.HOLD, f'trajectory complete; holding the last point '
                                             f'{self.g.hold_s:.1f} s')
            else:
                out.setpoint = sp
                self.last_setpoint = sp
                return out

        if self.phase == Phase.HOLD:
            sp = self.plan.sampler.at(self.plan.traj.duration)
            why = self._health(inp, sp)
            if why:
                self._enter(out, Phase.ABORT, why)
                return out
            if inp.now - self._t_hold0 >= self.g.hold_s:
                self._enter(out, Phase.DONE, 'hold complete; command stream stops, landing '
                                             'is the operator\'s')
                return out
            out.setpoint = sp
            self.last_setpoint = sp
        return out
