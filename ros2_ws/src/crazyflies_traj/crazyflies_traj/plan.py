"""
Frames and the flight plan: CSV frame -> the plant's world frame. Standard library only.

Frame modes (README "Frames"):

* ``absolute`` (DEFAULT): ``x, y`` ARE Motive world coordinates (Z-up, the
  frame of ``/<ns>/odom`` and ``/poses``). What the dry run prints is exactly what
  flies, and the whole geofence is checked before the vehicle is involved.
* ``relative``: ``x, y`` are offsets; the first row is placed at the vehicle's
  airborne ``x, y`` when the flight starts (anchor), so the world path is only
  known at trigger time and the geofence is re-checked then.

In BOTH modes ``z`` is the height above the pad z0 (``/<ns>/pad_z0``, the
pad-relative convention of the plant's clients): world ``z = z_csv + z0``. The
height is never re-anchored to wherever the vehicle happens to hover; a hover
at the wrong height is caught by the start-continuity check instead.

The approach (:func:`approach_plan`): when the airborne position is more than
``start_tol`` from the first setpoint the player does not refuse but first
streams a bounded move to it. (On the first flights the first setpoint of every
file sat 0.49 m below the HL hover, so each had to be flown ``relative``.)
The move is a 2-waypoint pchip in the vehicle's own world frame: with two rows
the Fritsch-Carlson end slopes are zero, so the curve is the cubic smoothstep
along the straight segment (peak speed ``1.5 d / T``, peak acceleration
``6 d / T^2``), checked like any trajectory against the limits and the
geofence. It then flows straight into the CSV stream with no gap: the firmware
levels out after 500 ms of silence (``supervisor.c``). With ``yaw: path`` it
also turns the nose from the measured yaw to the first setpoint's yaw at
``yaw_rate_max_dps`` and lasts at least as long as that turn.
"""

from dataclasses import dataclass
import math

from crazyflies_traj.sampler import (check_sampled_limits, path_length, path_yaw, Sampler,
                                     setpoint_table, wrap_pi, YawProfile, YawSpec)
from crazyflies_traj.trajectory import (check_geofence, check_limits, corner_accels, norm,
                                        segment_velocities, TrajError, Trajectory)

FRAMES = ('absolute', 'relative')
ZERO3 = (0.0, 0.0, 0.0)


def frame_offset(traj, frame, z0, anchor_xy=None):
    """Return the (dx, dy, z0) that maps CSV coordinates to world coordinates."""
    if frame == 'absolute':
        return (0.0, 0.0, z0)
    if frame == 'relative':
        if anchor_xy is None:
            raise TrajError('bad_frame', 'relative frame needs the airborne anchor x, y')
        x0, y0, _ = traj.points[0]
        return (anchor_xy[0] - x0, anchor_xy[1] - y0, z0)
    raise TrajError('bad_frame', f'frame must be one of {FRAMES}, got {frame!r}')


def make_sampler(traj, interp, offset, yaw, rate_hz, limits):
    """
    Return ``(Sampler, demand)``: the sampler with the yaw of ``yaw`` (a
    :class:`~crazyflies_traj.sampler.YawSpec`) attached, and the yaw rate (rad/s)
    the path demands (0 for ``fixed``).
    """
    yaw.check_sane()
    s = Sampler(traj, interp, offset)
    if yaw.yaw == 'fixed':
        s.yaw = YawProfile((0.0,), (math.radians(yaw.yaw_fixed_deg),))
        return s, 0.0
    s.yaw, demand = path_yaw(s, rate_hz, yaw, math.radians(limits.yaw_rate_max_dps))
    return s, demand


def check_offline(traj, limits, frame, interp, yaw=YawSpec(), rate_hz=50.0):
    """
    Every check possible before the vehicle is involved (the dry run / `traj check`).

    Returns the yaw rate (deg/s) the path demands (0 for ``yaw: fixed``); more
    than ``yaw_rate_max_dps`` is refused (``yaw_rate``).
    """
    check_limits(traj, limits, frame)
    demand = math.degrees(make_sampler(traj, interp, ZERO3, yaw, rate_hz, limits)[1])
    if demand > limits.yaw_rate_max_dps + 1e-9:
        raise TrajError('yaw_rate', f'{traj.source}: the path turns at {demand:.1f} deg/s > '
                                    f'yaw_rate_max_dps {limits.yaw_rate_max_dps:g} at '
                                    f'{rate_hz:g} Hz (slow the file down, or yaw: fixed)')
    # The smooth curve's own peaks; offset (0, 0, 0) keeps z pad-relative, and
    # for 'relative' the x, y part is re-done anchored at trigger time.
    probe = Sampler(traj, interp, offset=(0.0, 0.0, 0.0))
    if frame == 'absolute':
        check_sampled_limits(probe, limits)
    else:
        check_sampled_limits(_relative_probe(traj, interp, limits), limits)
    return demand


def _relative_probe(traj, interp, limits):
    # Place the first point at the box centre only to reuse the sampled checks
    # for speed / accel / z; the x, y geofence is authoritative at trigger time.
    cx = (limits.x_min + limits.x_max) / 2.0
    cy = (limits.y_min + limits.y_max) / 2.0
    x0, y0, _ = traj.points[0]
    xs = [p[0] - x0 for p in traj.points]
    ys = [p[1] - y0 for p in traj.points]
    # Centre the extent (already proven to fit) instead of the first point.
    dx = cx - (max(xs) + min(xs)) / 2.0 - x0
    dy = cy - (max(ys) + min(ys)) / 2.0 - y0
    return Sampler(traj, interp, offset=(dx, dy, 0.0))


@dataclass
class Plan:
    """A trajectory bound to the world: validated, anchored, ready to sample."""

    traj: object
    frame: str
    interp: str
    z0: float
    offset: tuple
    sampler: Sampler
    yaw: YawSpec = YawSpec()

    @property
    def first(self):
        """Return the first world setpoint position."""
        return self.sampler.at(0.0).pos

    @property
    def last(self):
        """Return the last world setpoint position (the hold point)."""
        return self.sampler.at(self.traj.duration).pos

    @property
    def first_yaw(self):
        """Return the yaw (rad) of the first setpoint."""
        return self.sampler.at(0.0).yaw

    def takeover_yaw(self, measured):
        """Return ``(yaw0, yaw1)``, the turn the approach makes: from the measured yaw
        for ``path`` (when known), none for ``fixed`` (today's behaviour)."""
        y1 = self.first_yaw
        return (measured if self.yaw.yaw == 'path' and measured is not None else y1), y1


def bind(traj, limits, frame, interp, z0, anchor_xy=None, yaw=YawSpec(), rate_hz=50.0):
    """
    Anchor the trajectory in the world and re-check what depends on the anchor.

    Refuses (TrajError) when the anchored waypoints, or the smooth curve between
    them, leave the geofence. Call at trigger time with the airborne x, y.
    """
    offset = frame_offset(traj, frame, z0, anchor_xy)
    check_geofence(traj, limits, offset=(offset[0], offset[1]))
    sampler = make_sampler(traj, interp, offset, yaw, rate_hz, limits)[0]
    check_sampled_limits(sampler, limits)
    return Plan(traj=traj, frame=frame, interp=interp, z0=z0, offset=offset, sampler=sampler,
                yaw=yaw)


def approach_plan(current, first, z0, limits, approach_v, yaw=(0.0, 0.0)):
    """
    Return the bound Plan of the move from the airborne ``current`` position to the
    ``first`` world setpoint: a 2-waypoint pchip (the cubic smoothstep along the
    straight segment), checked like any trajectory.

    The duration is the slower of the two bounds, so the peak speed ``1.5 d / T``
    stays at or under ``approach_v`` and the peak acceleration ``6 d / T^2`` at or
    under ``a_max``. Nothing is clamped: an ``approach_v`` the limits cannot hold
    is refused (``speed``), and a hover outside the box or inside the floor margin
    is refused as ``geofence`` / ``below_floor`` on ``approach: waypoint 0``.
    ``z`` is kept pad-relative so the z checks mean the same as for a CSV.

    ``yaw = (yaw0, yaw1)`` (rad): the nose turns the short way from ``yaw0`` to
    ``yaw1`` at ``yaw_rate_max_dps`` and then holds it, and the move lasts at
    least as long as the turn (equal yaws: no turn, as ``yaw: fixed``).
    """
    d = norm(tuple(first[a] - current[a] for a in range(3)))
    turn = wrap_pi(yaw[1] - yaw[0])
    t_turn = abs(turn) / math.radians(limits.yaw_rate_max_dps)
    duration = max(1.5 * d / approach_v, math.sqrt(6.0 * d / limits.a_max), t_turn)
    traj = Trajectory(times=(0.0, duration),
                      points=((current[0], current[1], current[2] - z0),
                              (first[0], first[1], first[2] - z0)),
                      source='approach')
    check_offline(traj, limits, 'absolute', 'pchip')
    plan = bind(traj, limits, 'absolute', 'pchip', z0)
    plan.sampler.yaw = (YawProfile((0.0, t_turn), (yaw[0], yaw[0] + turn)) if turn
                        else YawProfile((0.0,), (yaw[1],)))
    return plan


def summarize(traj, interp, offset=(0.0, 0.0, 0.0)):
    """Return the dry-run summary numbers as a dict (world frame when offset given)."""
    s = Sampler(traj, interp, offset=offset)
    if interp == 'linear':
        vmax = max(norm(v) for v in segment_velocities(traj))
        amax = max(corner_accels(traj))
    else:
        vmax = max(norm(s.at(traj.duration * k / 2000.0).vel) for k in range(2001))
        amax = max(norm(a) for i in range(len(traj) - 1) for a in s.acc_at_knots(i))
    pts = [s.at(t).pos for t in traj.times]
    lo = tuple(min(p[a] for p in pts) for a in range(3))
    hi = tuple(max(p[a] for p in pts) for a in range(3))
    return {
        'rows': len(traj),
        'duration_s': traj.duration,
        'path_length_m': path_length(s),
        'max_speed_mps': vmax,
        'max_accel_mps2': amax,
        'bbox_min': lo,
        'bbox_max': hi,
        't_shift_s': traj.t_shift,
    }


def format_summary(traj, interp, frame, rate_hz, offset=(0.0, 0.0, 0.0), z_label='above pad',
                   yaw_line=None):
    """Return the human dry-run summary block (list of lines)."""
    sm = summarize(traj, interp, offset)
    n = len(setpoint_table(Sampler(traj, interp, offset), rate_hz))
    lo, hi = sm['bbox_min'], sm['bbox_max']
    lines = [
        f'trajectory {traj.source}: {sm["rows"]} waypoints, frame {frame}, interp {interp}',
        f'  duration      {sm["duration_s"]:.3f} s'
        + (f' (t shifted by {sm["t_shift_s"]:g} s)' if sm['t_shift_s'] else ''),
        f'  path length   {sm["path_length_m"]:.3f} m',
        f'  max speed     {sm["max_speed_mps"]:.3f} m/s',
        f'  max accel     {sm["max_accel_mps2"]:.3f} m/s^2',
        f'  bbox x        [{lo[0]:.3f}, {hi[0]:.3f}] m',
        f'  bbox y        [{lo[1]:.3f}, {hi[1]:.3f}] m',
        f'  bbox z        [{lo[2]:.3f}, {hi[2]:.3f}] m ({z_label})',
        f'  setpoints     {n} at {rate_hz:g} Hz (then hold the last point)',
    ]
    if yaw_line:
        lines.append(f'  yaw           {yaw_line}')
    return lines


def format_setpoints(table, acc_ff, yaw=False):
    """Return one line per setpoint: t, pos, vel (acc when fed forward; yaw deg, yaw rate deg/s)."""
    out = ['#     t        x        y        z       vx       vy       vz'
           + ('       ax       ay       az' if acc_ff else '')
           + ('      yaw  yawrate' if yaw else '')]
    for sp in table:
        acc = sp.acc if acc_ff else ()
        y = (math.degrees(sp.yaw), math.degrees(sp.yaw_rate)) if yaw else ()
        vals = (sp.t,) + tuple(sp.pos) + tuple(sp.vel) + tuple(acc) + y
        out.append(' '.join(_fmt(v) for v in vals))
    return out


def _fmt(v):
    s = f'{v:8.3f}'
    return '   0.000' if s == '  -0.000' else s
