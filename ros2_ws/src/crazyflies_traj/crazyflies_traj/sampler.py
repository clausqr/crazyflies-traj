"""
Setpoint sampler: turn a :class:`~crazyflies_traj.trajectory.Trajectory` into
position / velocity / acceleration setpoints. Standard library only.

Two interpolations:

* ``linear`` (default): straight segments between waypoints; the velocity
  feed-forward is the constant segment velocity, the acceleration feed-forward
  is 0 (a polyline's true acceleration is an impulse at every corner).
* ``pchip``: per-axis monotone cubic Hermite (Fritsch-Carlson slopes), with zero
  velocity at the first and last waypoint (the vehicle starts hovering and ends
  holding). Monotone per axis means each axis stays inside the range of its two
  waypoints, so the curve can never leave an axis-aligned geofence box the
  waypoints are inside. Its peak speed and acceleration CAN exceed the polyline
  values, so :func:`check_sampled_limits` checks the curve itself.

Time is the trajectory time ``t`` in seconds from the first waypoint. Before
0 the sampler returns the first point, after the last it returns the last,
both with zero velocity and acceleration (hold).

Yaw (README "Yaw"): ``fixed`` (default) commands ``yaw_fixed_deg`` throughout.
``path`` commands the heading of the sampled velocity, ``atan2(vy, vx)``, so it
follows the interpolation the position uses. It is tabulated on the setpoint
grid (:func:`path_yaw`): below ``yaw_min_speed`` (horizontal) the last valid
heading is held (before the first one, the first one), the heading is
unwrapped, and every step is slewed to ``yaw_rate_max_dps``; between grid
points the yaw is linear and the yaw rate is that segment's slope.
"""

import bisect
from dataclasses import dataclass
import math

from crazyflies_traj.trajectory import check_point_in_box, norm, TrajError

INTERPOLATIONS = ('linear', 'pchip')
YAW_MODES = ('fixed', 'path')
ZERO = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Setpoint:
    """One command: trajectory time ``t``, world-frame pos / vel / acc, yaw (rad) and its rate (rad/s)."""

    t: float
    pos: tuple
    vel: tuple
    acc: tuple
    yaw: float = 0.0
    yaw_rate: float = 0.0


@dataclass(frozen=True)
class YawSpec:
    """The player's yaw parameters (the field names are the node parameter names)."""

    yaw: str = 'fixed'          # 'fixed' or 'path'
    yaw_fixed_deg: float = 0.0  # the yaw of 'fixed' (and of 'path' when nothing moves horizontally)
    yaw_min_speed: float = 0.05  # m/s horizontal; below it 'path' holds the last heading

    def check_sane(self):
        """Refuse an unknown mode or a non-positive speed floor."""
        if self.yaw not in YAW_MODES:
            raise TrajError('bad_yaw', f'yaw must be one of {YAW_MODES}, got {self.yaw!r}')
        if not (math.isfinite(self.yaw_fixed_deg) and math.isfinite(self.yaw_min_speed)
                and self.yaw_min_speed > 0.0):
            raise TrajError('bad_yaw', 'yaw_fixed_deg must be finite and yaw_min_speed > 0')


class YawProfile:
    """Piecewise-linear yaw over trajectory time (rad, unwrapped); held outside its times."""

    def __init__(self, times, yaws):
        self.times = list(times)
        self.yaws = list(yaws)

    def at(self, t):
        """Return ``(yaw, yaw_rate)`` at ``t``: the rate is the slope of the segment flown."""
        ts, ys = self.times, self.yaws
        if t < ts[0] or t >= ts[-1]:
            return (ys[0] if t < ts[0] else ys[-1]), 0.0
        i = bisect.bisect_right(ts, t) - 1
        rate = (ys[i + 1] - ys[i]) / (ts[i + 1] - ts[i])
        return ys[i] + rate * (t - ts[i]), rate


def wrap_pi(a):
    """Return ``a`` wrapped to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _pchip_slopes(times, ys):
    """Fritsch-Carlson monotone slopes for one axis, zero at both ends."""
    n = len(times)
    h = [times[i + 1] - times[i] for i in range(n - 1)]
    d = [(ys[i + 1] - ys[i]) / h[i] for i in range(n - 1)]
    m = [0.0] * n
    for k in range(1, n - 1):
        if d[k - 1] * d[k] <= 0.0:
            m[k] = 0.0
        else:
            w1 = 2.0 * h[k] + h[k - 1]
            w2 = h[k] + 2.0 * h[k - 1]
            m[k] = (w1 + w2) / (w1 / d[k - 1] + w2 / d[k])
    return m


class Sampler:
    """Evaluate a trajectory at any time, in its own (CSV) frame plus an offset."""

    def __init__(self, traj, interp='linear', offset=ZERO, yaw=None):
        if interp not in INTERPOLATIONS:
            raise TrajError('bad_interp', f'interp must be one of {INTERPOLATIONS}, got {interp!r}')
        self.traj = traj
        self.interp = interp
        self.offset = tuple(offset)
        self.yaw = yaw if yaw is not None else YawProfile((0.0,), (0.0,))
        self._t = traj.times
        self._p = traj.points
        if interp == 'pchip':
            self._m = [_pchip_slopes(self._t, [p[a] for p in self._p]) for a in range(3)]

    @property
    def duration(self):
        """Return the trajectory duration (s)."""
        return self.traj.duration

    def _shift(self, p):
        o = self.offset
        return (p[0] + o[0], p[1] + o[1], p[2] + o[2])

    def at(self, t):
        """Return the :class:`Setpoint` at trajectory time ``t`` (world frame), with its yaw."""
        sp = self._at(t)
        yaw, rate = self.yaw.at(t)
        return Setpoint(sp.t, sp.pos, sp.vel, sp.acc, yaw, rate)

    def _at(self, t):
        times = self._t
        if t <= 0.0:
            return Setpoint(t, self._shift(self._p[0]), ZERO, ZERO)
        if t >= times[-1]:
            return Setpoint(t, self._shift(self._p[-1]), ZERO, ZERO)
        i = bisect.bisect_right(times, t) - 1          # times[i] <= t < times[i + 1]
        t0, t1 = times[i], times[i + 1]
        h = t1 - t0
        p0, p1 = self._p[i], self._p[i + 1]
        if t == t0:
            pos = p0                                    # exact at every waypoint
        if self.interp == 'linear':
            s = (t - t0) / h
            if t != t0:
                pos = tuple(p0[a] + s * (p1[a] - p0[a]) for a in range(3))
            vel = tuple((p1[a] - p0[a]) / h for a in range(3))
            return Setpoint(t, self._shift(pos), vel, ZERO)
        s = (t - t0) / h
        s2, s3 = s * s, s * s * s
        pos_c, vel_c, acc_c = [], [], []
        for a in range(3):
            y0, y1 = p0[a], p1[a]
            m0, m1 = self._m[a][i] * h, self._m[a][i + 1] * h
            y = ((2 * s3 - 3 * s2 + 1) * y0 + (s3 - 2 * s2 + s) * m0
                 + (-2 * s3 + 3 * s2) * y1 + (s3 - s2) * m1)
            # Monotone per axis: y is mathematically inside [y0, y1]. The clamp
            # only removes float round-off (1e-16); a real excursion means the
            # slopes are wrong and is raised, never clamped away.
            lo, hi = min(y0, y1), max(y0, y1)
            if not lo - 1e-9 <= y <= hi + 1e-9:
                raise TrajError('internal', f'pchip left [{lo}, {hi}] on axis {a}: {y}')
            pos_c.append(min(max(y, lo), hi))
            vel_c.append(((6 * s2 - 6 * s) * y0 + (3 * s2 - 4 * s + 1) * m0
                          + (-6 * s2 + 6 * s) * y1 + (3 * s2 - 2 * s) * m1) / h)
            acc_c.append(((12 * s - 6) * y0 + (6 * s - 4) * m0
                          + (-12 * s + 6) * y1 + (6 * s - 2) * m1) / (h * h))
        if t == t0:
            pos_c = list(p0)
        return Setpoint(t, self._shift(tuple(pos_c)), tuple(vel_c), tuple(acc_c))

    def acc_at_knots(self, i):
        """Return the pchip acceleration at both ends of segment ``i`` (limits check)."""
        t0, t1 = self._t[i], self._t[i + 1]
        h = t1 - t0
        out = []
        for s in (0.0, 1.0):
            acc = []
            for a in range(3):
                y0, y1 = self._p[i][a], self._p[i + 1][a]
                m0, m1 = self._m[a][i] * h, self._m[a][i + 1] * h
                acc.append(((12 * s - 6) * y0 + (6 * s - 4) * m0
                            + (-12 * s + 6) * y1 + (6 * s - 2) * m1) / (h * h))
            out.append(tuple(acc))
        return out


def sample_times(duration, rate_hz):
    """
    Return the command times ``k / rate`` for ``k = 0..n``, ending exactly at ``duration``.

    ``n = floor(duration * rate)`` (with a 1e-9 guard against float fuzz); when
    ``n / rate`` falls short of ``duration`` the exact end time is appended, so
    the last setpoint is always the last waypoint.
    """
    if not (math.isfinite(rate_hz) and rate_hz > 0.0):
        raise TrajError('bad_rate', f'rate_hz must be > 0, got {rate_hz!r}')
    n = int(math.floor(duration * rate_hz + 1e-9))
    ts = [k / rate_hz for k in range(n + 1)]
    if duration - ts[-1] > 1e-9:
        ts.append(duration)
    else:
        ts[-1] = duration
    return ts


def path_yaw(sampler, rate_hz, spec, rate_max, yaw0=None):
    """
    Return ``(YawProfile, demand)``: the ``path`` yaw on the setpoint grid and the
    largest yaw rate (rad/s) the path itself asks for while moving.

    ``demand`` is the heading change between neighbouring grid points that both
    move at least ``yaw_min_speed``, over the grid step: what the offline check
    holds against ``yaw_rate_max_dps``. A turn across a stop (speed below the
    floor) is not demanded by the path; the slew takes it at ``rate_max``.
    ``yaw0`` (rad) starts the slew (default: the first valid heading).
    """
    ts = sample_times(sampler.duration, rate_hz)
    heads = []
    for t in ts:
        v = sampler.at(t).vel
        heads.append(math.atan2(v[1], v[0]) if math.hypot(v[0], v[1]) >= spec.yaw_min_speed
                     else None)
    valid = [h for h in heads if h is not None]
    if not valid:
        y = math.radians(spec.yaw_fixed_deg) if yaw0 is None else yaw0
        return YawProfile((0.0,), (y,)), 0.0
    demand = 0.0
    for k in range(len(ts) - 1):
        if heads[k] is not None and heads[k + 1] is not None:
            demand = max(demand, abs(wrap_pi(heads[k + 1] - heads[k])) / (ts[k + 1] - ts[k]))
    held = valid[0]
    yaws = [valid[0] if yaw0 is None else yaw0]
    for k in range(1, len(ts)):
        held = heads[k] if heads[k] is not None else held
        step = rate_max * (ts[k] - ts[k - 1])
        yaws.append(yaws[-1] + min(max(wrap_pi(held - yaws[-1]), -step), step))
    return YawProfile(ts, yaws), demand


def setpoint_table(sampler, rate_hz):
    """Return every setpoint the player would stream at ``rate_hz`` (no hold)."""
    return [sampler.at(t) for t in sample_times(sampler.duration, rate_hz)]


def check_sampled_limits(sampler, limits, per_segment=64):
    """
    Refuse a smooth curve whose own speed, acceleration or position breaks a limit.

    Linear interpolation is fully covered by
    :func:`~crazyflies_traj.trajectory.check_limits`. For ``pchip`` this
    samples ``per_segment`` points in every segment for speed and position
    (the per-axis monotone property already bounds position; this is the
    witness), and evaluates the acceleration exactly at the segment ends (the
    acceleration of a cubic is affine in time, and the norm of an affine vector
    function is convex, so its maximum is at an end).
    """
    if sampler.interp == 'linear':
        return
    tr = sampler.traj
    o = sampler.offset
    for i in range(len(tr) - 1):
        t0, t1 = tr.times[i], tr.times[i + 1]
        for k in range(per_segment + 1):
            sp = sampler.at(t0 + (t1 - t0) * k / per_segment)
            v = norm(sp.vel)
            if v > limits.v_max + 1e-9:
                raise TrajError('speed', f'{tr.source}: pchip speed {v:.3f} m/s at t '
                                         f'{sp.t:.3f} s exceeds v_max {limits.v_max:.3f} '
                                         '(the smooth curve peaks above the polyline; '
                                         'use linear or slow the segment)')
            check_point_in_box((sp.pos[0], sp.pos[1]), sp.pos[2] - o[2], limits,
                               f'{tr.source}: pchip point at t {sp.t:.3f} s')
        for acc in sampler.acc_at_knots(i):
            a = norm(acc)
            if a > limits.a_max + 1e-9:
                raise TrajError('accel', f'{tr.source}: pchip acceleration {a:.3f} m/s^2 in '
                                         f'segment {i} exceeds a_max {limits.a_max:.3f}')


def path_length(sampler, per_segment=64):
    """Return the path length (m): exact for linear, sampled for pchip."""
    tr = sampler.traj
    if sampler.interp == 'linear':
        return sum(norm(tuple(tr.points[i + 1][a] - tr.points[i][a] for a in range(3)))
                   for i in range(len(tr) - 1))
    total = 0.0
    prev = sampler.at(0.0).pos
    for i in range(len(tr) - 1):
        t0, t1 = tr.times[i], tr.times[i + 1]
        for k in range(1, per_segment + 1):
            p = sampler.at(t0 + (t1 - t0) * k / per_segment).pos
            total += norm(tuple(p[a] - prev[a] for a in range(3)))
            prev = p
    return total
