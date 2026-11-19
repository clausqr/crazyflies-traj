"""CSV trajectory: waypoints x, y, z, t linearly interpolated. Standard library only."""

import argparse
import math
import sys
from typing import NamedTuple

YAW_MODES = ('fixed', 'path')


class Setpoint(NamedTuple):
    """World frame (z above the pad), yaw in rad."""
    pos: tuple
    vel: tuple
    acc: tuple
    yaw: float


class Trajectory:
    """Waypoints (times, points) with one yaw per segment."""

    def __init__(self, times, points, yaw='fixed', yaw_deg=0.0):
        if yaw not in YAW_MODES:
            raise ValueError(f'unknown yaw mode {yaw!r}, use one of {YAW_MODES}')
        self.times = list(times)
        self.points = [tuple(p) for p in points]
        n = len(self.times) - 1
        heads = [math.radians(yaw_deg)] * n
        if yaw == 'path':
            heads = [None if math.hypot(b[0] - a[0], b[1] - a[1]) < 1e-6
                     else math.atan2(b[1] - a[1], b[0] - a[0])
                     for a, b in zip(self.points, self.points[1:])]
            first = next((h for h in heads if h is not None), math.radians(yaw_deg))
            prev = first
            for i, h in enumerate(heads):
                if h is None:
                    h = prev
                h += 2 * math.pi * round((prev - h) / (2 * math.pi))
                heads[i] = prev = h
        self.yaws = heads

    @property
    def duration(self):
        return self.times[-1]

    def at(self, t):
        """Setpoint at time t (clamped to the trajectory); velocity is that of the segment."""
        t = min(max(t, 0.0), self.duration)
        i = 0
        while i < len(self.times) - 2 and t >= self.times[i + 1]:
            i += 1
        t0, t1 = self.times[i], self.times[i + 1]
        a, b = self.points[i], self.points[i + 1]
        s = (t - t0) / (t1 - t0)
        pos = tuple(p + s * (q - p) for p, q in zip(a, b))
        vel = tuple((q - p) / (t1 - t0) for p, q in zip(a, b))
        if t <= 0.0 or t >= self.duration:
            vel = (0.0, 0.0, 0.0)
        return Setpoint(pos, vel, (0.0, 0.0, 0.0), self.yaws[i])


def parse_csv(text, speedup=1.0, yaw='fixed', yaw_deg=0.0):
    """Parse CSV text (optional x,y,z,t header in any order, else that order) into a Trajectory."""
    if not (math.isfinite(speedup) and speedup > 0):
        raise ValueError(f'speedup must be finite and > 0, got {speedup}')
    lines = [ln.strip() for ln in text.splitlines()]
    lines = [ln for ln in lines if ln and not ln.startswith('#')]
    order = ['x', 'y', 'z', 't']
    if lines:
        try:
            [float(f) for f in lines[0].split(',')]
        except ValueError:
            order = [f.strip().lower() for f in lines.pop(0).split(',')]
            if sorted(order) != ['t', 'x', 'y', 'z']:
                raise ValueError(f'header must name x, y, z, t, got {order}')
    rows = []
    for ln in lines:
        fields = ln.split(',')
        if len(fields) != 4:
            raise ValueError(f'expected 4 fields, got {len(fields)}: {ln!r}')
        vals = [float(f) for f in fields]
        if not all(math.isfinite(v) for v in vals):
            raise ValueError(f'non-finite value: {ln!r}')
        rows.append(dict(zip(order, vals)))
    if len(rows) < 2:
        raise ValueError('need at least 2 rows')
    times = [r['t'] for r in rows]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError('t must be strictly increasing')
    times = [(t - times[0]) / speedup for t in times]
    return Trajectory(times, [(r['x'], r['y'], r['z']) for r in rows], yaw, yaw_deg)


def load_csv(path, speedup=1.0, yaw='fixed', yaw_deg=0.0):
    """Read a CSV file with parse_csv."""
    with open(path) as f:
        return parse_csv(f.read(), speedup, yaw, yaw_deg)


def main(argv=None):
    """Print the sampled trajectory: t x y z vx vy vz yaw_deg."""
    ap = argparse.ArgumentParser(description=main.__doc__)
    ap.add_argument('file')
    ap.add_argument('--rate', type=float, default=50.0)
    ap.add_argument('--yaw', choices=YAW_MODES, default='fixed')
    ap.add_argument('--speedup', type=float, default=1.0)
    a = ap.parse_args(argv)
    tr = load_csv(a.file, a.speedup, a.yaw)
    for k in range(int(tr.duration * a.rate) + 2):
        t = min(k / a.rate, tr.duration)
        s = tr.at(t)
        print(f'{t:.3f}', *(f'{v:.4f}' for v in s.pos + s.vel),
              f'{math.degrees(s.yaw):.2f}')
        if t >= tr.duration:
            break
    return 0


if __name__ == '__main__':
    sys.exit(main())
