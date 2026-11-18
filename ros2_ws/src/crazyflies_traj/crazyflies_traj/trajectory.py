"""
CSV trajectory loader and validator. Standard library only (no ROS, no numpy).

File format (see README "CSV format"):

* One header row naming the four columns ``x, y, z, t`` (any order, case and
  surrounding blanks ignored). A headerless file is accepted ONLY with
  ``header=False`` (CLI ``--no-header``), and then the column order is fixed to
  ``x, y, z, t``.
* Units: meters and seconds. ``x, y`` are in the frame selected by the frame
  mode (``absolute``: Motive world, Z-up, the frame the plant's ``/<ns>/odom`` and
  ``/poses`` use; ``relative``: offsets from the first row, anchored at the
  airborne position when the flight starts). ``z`` is ALWAYS the height above
  the pad z0 (``/<ns>/pad_z0``), the pad-relative convention of the plant's clients.
* ``t`` starts at exactly 0 and strictly increases. A file that starts at a
  nonzero ``t`` is refused unless ``normalize_t=True`` (CLI ``--normalize-t``),
  which shifts every ``t`` by the first one.
* Blank lines and lines whose first non-blank character is ``#`` are ignored.

Every refusal raises :class:`TrajError` with a stable ``code`` (the refusal
table in the README lists them) and a human message naming the row.
"""

import csv
from dataclasses import dataclass
import io
import math

COLUMNS = ('x', 'y', 'z', 't')

# Row numbers in messages are 1-based FILE line numbers, so an operator can jump
# straight to the offending line.


class TrajError(ValueError):
    """A trajectory was refused. ``code`` is stable and machine-checkable."""

    def __init__(self, code, msg):
        super().__init__(f'{code}: {msg}')
        self.code = code
        self.msg = msg


@dataclass(frozen=True)
class Trajectory:
    """A validated piecewise trajectory: ``times[i]`` -> ``points[i]`` (x, y, z)."""

    times: tuple
    points: tuple
    source: str = '<memory>'
    t_shift: float = 0.0          # seconds removed by normalize_t (0 when not used)
    speedup: float = 1.0          # every file t divided by this (1 = as written)

    @property
    def duration(self):
        """Return the time of the last waypoint (the first is 0)."""
        return self.times[-1]

    def __len__(self):
        return len(self.times)


@dataclass(frozen=True)
class Limits:
    """
    Refusal limits. Defaults are the envelope sized to the first spherical Lissajous
    flights (README "Limits").

    The geofence is an axis-aligned box: ``x`` and ``y`` in the Motive world
    frame (Z-up, the plant's ``/<ns>/odom`` and ``/poses`` frame), ``z`` as height
    above the pad z0 (so it is checkable offline, before z0 is known).
    """

    x_min: float = -1.75
    x_max: float = 1.75
    y_min: float = -1.75
    y_max: float = 1.75
    z_floor: float = 0.20      # min height above pad z0 (m): the floor margin
    z_max: float = 2.00        # max height above pad z0 (m)
    v_max: float = 2.50        # m/s (the flown fast Lissajous peaks at 1.91)
    a_max: float = 4.00        # m/s^2
    start_tol: float = 0.15    # m, 3D distance first setpoint <-> airborne position
    max_duration: float = 120.0  # s, one battery-safe leg
    yaw_rate_max_dps: float = 90.0  # deg/s, the yaw slew and the most a 'yaw: path' file may demand

    def check_sane(self):
        """Refuse limits that are themselves nonsense (empty box, non-positive rates)."""
        vals = (self.x_min, self.x_max, self.y_min, self.y_max, self.z_floor, self.z_max,
                self.v_max, self.a_max, self.start_tol, self.max_duration, self.yaw_rate_max_dps)
        if not all(math.isfinite(v) for v in vals):
            raise TrajError('bad_limits', 'every limit must be finite')
        if not (self.x_min < self.x_max and self.y_min < self.y_max
                and 0.0 < self.z_floor < self.z_max):
            raise TrajError('bad_limits', f'empty or inverted geofence box {self}')
        if min(self.v_max, self.a_max, self.start_tol, self.max_duration,
               self.yaw_rate_max_dps) <= 0.0:
            raise TrajError('bad_limits', 'v_max, a_max, start_tol, max_duration, '
                                          'yaw_rate_max_dps must be > 0')


def _data_lines(text):
    """Yield ``(line_no, line)`` for every non-blank, non-comment line."""
    for no, line in enumerate(io.StringIO(text), start=1):
        s = line.strip()
        if not s or s.startswith('#'):
            continue
        yield no, line


def parse_csv(text, header=True, normalize_t=False, source='<memory>', speedup=1.0):
    """
    Parse and structurally validate CSV text into a :class:`Trajectory`.

    Checks: header, 4 numeric finite fields per row, at least 2 rows, no
    duplicate row, ``t`` strictly increasing, ``t`` starting at 0 (or shifted
    with ``normalize_t``). ``speedup`` divides every ``t`` (2 = fly the file in
    half the time; the speed and acceleration limits then see the real, faster
    motion). Kinematic and geofence limits are separate (:func:`check_limits`),
    because the geofence depends on the frame mode.
    """
    if not (math.isfinite(speedup) and speedup > 0):
        raise TrajError('bad_speedup', f'{source}: speedup must be a finite number > 0, '
                                       f'got {speedup!r}')
    lines = list(_data_lines(text))
    order = list(COLUMNS)
    if header:
        if not lines:
            raise TrajError('too_few_rows', f'{source}: no header and no data rows')
        no, line = lines[0]
        names = [c.strip().lower() for c in next(csv.reader([line]))]
        if sorted(names) != sorted(COLUMNS) or len(names) != 4:
            looks_numeric = all(_is_number(c) for c in names)
            hint = (' (the first row looks numeric: pass --no-header / header:=false for a '
                    'headerless file)') if looks_numeric else ''
            raise TrajError('bad_header', f'{source}:{no}: header must name exactly the columns '
                                          f'x, y, z, t, got {names}{hint}')
        order = names
        lines = lines[1:]

    rows = []
    for no, line in lines:
        fields = [c.strip() for c in next(csv.reader([line]))]
        if len(fields) != 4:
            raise TrajError('bad_row', f'{source}:{no}: expected 4 fields (x, y, z, t), '
                                       f'got {len(fields)}: {line.strip()!r}')
        vals = {}
        for name, raw in zip(order, fields):
            try:
                v = float(raw)
            except ValueError:
                raise TrajError('not_numeric', f'{source}:{no}: {name}={raw!r} is not a number')
            if not math.isfinite(v):
                raise TrajError('non_finite', f'{source}:{no}: {name}={raw!r} is not finite')
            vals[name] = v
        rows.append((no, vals['t'], (vals['x'], vals['y'], vals['z'])))

    if len(rows) < 2:
        raise TrajError('too_few_rows', f'{source}: {len(rows)} data row(s); a trajectory '
                                        'needs at least 2')

    for (no0, t0, p0), (no1, t1, p1) in zip(rows, rows[1:]):
        if t1 == t0 and p1 == p0:
            raise TrajError('duplicate_row', f'{source}:{no1}: duplicates line {no0}')
        if not t1 > t0:
            raise TrajError('non_monotone_t', f'{source}:{no1}: t={t1:g} does not increase '
                                              f'past t={t0:g} (line {no0})')

    t_first = rows[0][1]
    shift = 0.0
    if t_first != 0.0:
        if not normalize_t:
            raise TrajError('t_not_zero', f'{source}:{rows[0][0]}: first t is {t_first:g}, '
                                          'must be 0 (pass --normalize-t / normalize_t:=true '
                                          'to shift every t by the first one)')
        shift = t_first

    times = tuple((r[1] - shift) / speedup for r in rows)
    points = tuple(r[2] for r in rows)
    return Trajectory(times=times, points=points, source=source, t_shift=shift, speedup=speedup)


def load_csv(path, header=True, normalize_t=False, speedup=1.0):
    """Read ``path`` and :func:`parse_csv` it."""
    try:
        with open(path, encoding='utf-8') as f:
            text = f.read()
    except OSError as exc:
        raise TrajError('unreadable', f'{path}: {exc.strerror or exc}')
    return parse_csv(text, header=header, normalize_t=normalize_t, source=str(path),
                     speedup=speedup)


def _is_number(s):
    try:
        float(s)
    except ValueError:
        return False
    return True


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def norm(v):
    """Return the Euclidean norm of a 3-vector."""
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def segment_velocities(traj):
    """Return the constant velocity of each linear segment (len = rows - 1)."""
    out = []
    for i in range(len(traj) - 1):
        h = traj.times[i + 1] - traj.times[i]
        d = _sub(traj.points[i + 1], traj.points[i])
        out.append((d[0] / h, d[1] / h, d[2] / h))
    return out


def corner_accels(traj):
    """
    Return the finite-difference acceleration at every waypoint, rest to rest.

    The vehicle starts hovering and ends holding, so the velocity sequence is
    ``0, v_0, ..., v_{n-2}, 0``. The acceleration at waypoint ``i`` is the
    velocity change there over the mean of the adjacent segment durations (the
    first and last use half their only segment). For a densely sampled smooth
    curve this converges to the true acceleration (a circle sampled at 10 deg
    reads v^2/r within 1 percent), and it is independent of the interpolation.
    """
    vs = segment_velocities(traj)
    hs = [traj.times[i + 1] - traj.times[i] for i in range(len(traj) - 1)]
    zero = (0.0, 0.0, 0.0)
    vel = [zero] + vs + [zero]
    win = [hs[0] / 2.0] + [(hs[i - 1] + hs[i]) / 2.0 for i in range(1, len(hs))] + [hs[-1] / 2.0]
    return [norm(_sub(vel[i + 1], vel[i])) / win[i] for i in range(len(traj))]


def check_limits(traj, limits, frame='absolute'):
    """
    Refuse a trajectory that breaks a kinematic or (frame-dependent) geofence limit.

    ``absolute``: every waypoint's x, y must lie in the world box and z (height
    above pad) in ``[z_floor, z_max]``, so the check is complete offline.
    ``relative``: x, y are only known once anchored, so offline this checks z
    and that the x, y EXTENT fits in the box; the node re-checks the anchored
    waypoints with :func:`check_geofence` at trigger time.
    Linear-segment speed and corner acceleration are checked here; the smooth
    interpolation's own peaks are checked by the sampler (it can exceed both).
    """
    limits.check_sane()
    if traj.duration > limits.max_duration:
        raise TrajError('too_long', f'{traj.source}: duration {traj.duration:.2f} s exceeds '
                                    f'max_duration {limits.max_duration:.2f} s')
    for i, p in enumerate(traj.points):
        check_z(p[2], limits, _waypoint(traj, i))
    if frame == 'absolute':
        check_geofence(traj, limits, offset=(0.0, 0.0))
    elif frame == 'relative':
        xs = [p[0] for p in traj.points]
        ys = [p[1] for p in traj.points]
        if max(xs) - min(xs) > limits.x_max - limits.x_min or \
                max(ys) - min(ys) > limits.y_max - limits.y_min:
            raise TrajError('geofence', f'{traj.source}: x/y extent {max(xs) - min(xs):.2f} x '
                                        f'{max(ys) - min(ys):.2f} m does not fit the geofence '
                                        'box at any anchor')
    else:
        raise TrajError('bad_frame', f"frame must be 'absolute' or 'relative', got {frame!r}")

    for i, v in enumerate(segment_velocities(traj)):
        s = norm(v)
        if s > limits.v_max + 1e-9:
            raise TrajError('speed', f'{traj.source}: segment {i} (t {traj.times[i]:g} -> '
                                     f'{traj.times[i + 1]:g} s) needs {s:.3f} m/s > v_max '
                                     f'{limits.v_max:.3f}')
    for i, a in enumerate(corner_accels(traj)):
        if a > limits.a_max + 1e-9:
            raise TrajError('accel', f'{traj.source}: waypoint {i} (t {traj.times[i]:g} s) needs '
                                     f'{a:.3f} m/s^2 > a_max {limits.a_max:.3f}')


def _waypoint(traj, i):
    return f'{traj.source}: waypoint {i} (t {traj.times[i]:g} s)'


def check_z(z_rel, limits, what):
    """Refuse a height above pad below the floor margin or above z_max (the ONE z check)."""
    if z_rel < limits.z_floor - 1e-9:
        raise TrajError('below_floor', f'{what}: z {z_rel:.3f} m above pad is below the floor '
                                       f'margin {limits.z_floor:.3f} m')
    if z_rel > limits.z_max + 1e-9:
        raise TrajError('geofence', f'{what}: z {z_rel:.3f} m above pad exceeds z_max '
                                    f'{limits.z_max:.3f} m')


def check_xy(x, y, limits, what):
    """Refuse a world x, y outside the geofence box (the ONE x, y check)."""
    if not (limits.x_min - 1e-9 <= x <= limits.x_max + 1e-9
            and limits.y_min - 1e-9 <= y <= limits.y_max + 1e-9):
        raise TrajError('geofence', f'{what}: (x {x:.3f}, y {y:.3f}) outside the geofence '
                                    f'x [{limits.x_min:g}, {limits.x_max:g}] y '
                                    f'[{limits.y_min:g}, {limits.y_max:g}]')


def check_point_in_box(p_xy_world, z_rel, limits, what):
    """Refuse one world point (x, y world; z above pad) outside the geofence."""
    check_xy(p_xy_world[0], p_xy_world[1], limits, what)
    check_z(z_rel, limits, what)


def check_geofence(traj, limits, offset):
    """Refuse any waypoint that, shifted by the x, y ``offset``, leaves the box."""
    for i, p in enumerate(traj.points):
        check_point_in_box((p[0] + offset[0], p[1] + offset[1]), p[2], limits,
                           _waypoint(traj, i))


def check_start(first_world, current, tol):
    """Refuse when the first setpoint is more than ``tol`` (3D) from the vehicle."""
    d = norm(_sub(first_world, current))
    if not d <= tol:
        raise TrajError('start_far', f'first setpoint ({first_world[0]:.3f}, {first_world[1]:.3f}, '
                                     f'{first_world[2]:.3f}) is {d:.3f} m from the airborne '
                                     f'position ({current[0]:.3f}, {current[1]:.3f}, '
                                     f'{current[2]:.3f}); start_tol {tol:.3f} m')
    return d
