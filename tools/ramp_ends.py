#!/usr/bin/env python3
"""
Write the fleet circles with their ends ramped from and to rest.

    python3 tools/ramp_ends.py        # run from the repo root

Reads trajectories/fleet/src/circle_*.csv as given (headerless x, y, z, t rows,
constant speed, the first row already at full speed) and writes
trajectories/fleet/circle_*.csv: every x, y, z is kept, t is re-timed so the
speed follows a cosine ramp from rest over the first RAMP_S seconds and back
to rest over the last RAMP_S seconds (the same path, RAMP_S longer in all).
This way the vehicle starts and stops at rest. The originals are kept.
"""
import csv
import glob
import math
import os

SRC = 'trajectories/fleet/src'
DST = 'trajectories/fleet'
RAMP_S = 4.0


def _ramped_distance(tau):
    """Distance covered (in units of the full speed) after tau seconds of a cosine ramp."""
    return (tau - RAMP_S / math.pi * math.sin(math.pi * tau / RAMP_S)) / 2.0


def _ramp_time(dist):
    """Invert _ramped_distance by bisection (dist in [0, RAMP_S / 2])."""
    lo, hi = 0.0, RAMP_S
    for _ in range(60):
        mid = (lo + hi) / 2.0
        if _ramped_distance(mid) < dist:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


def retime(t, t_end):
    """Map the original time t (constant speed, 0..t_end) to the ramped timeline."""
    half = RAMP_S / 2.0                       # distance covered by each ramp
    if t <= half:
        return _ramp_time(t)
    if t >= t_end - half:
        return t_end + RAMP_S - _ramp_time(t_end - t)
    return t + half


def main():
    for src in sorted(glob.glob(os.path.join(SRC, 'circle_*.csv'))):
        rows = []
        with open(src, encoding='utf-8') as f:
            for line in f:
                s = line.strip()
                if not s or s.startswith('#'):
                    continue
                x, y, z, t = (float(c) for c in s.split(','))
                rows.append((x, y, z, t))
        t_end = rows[-1][3]
        dst = os.path.join(DST, os.path.basename(src))
        with open(dst, 'w', newline='', encoding='utf-8') as f:
            f.write(f'# {os.path.basename(src)} as given in fleet/src (same x, y, z), ends ramped '
                    f'from and to rest over {RAMP_S:g} s by tools/ramp_ends.py\n')
            w = csv.writer(f)
            w.writerow(['x', 'y', 'z', 't'])
            for x, y, z, t in rows:
                w.writerow([f'{x:.8f}', f'{y:.8f}', f'{z:.8f}', f'{retime(t, t_end):.6f}'])
        print(f'{dst}: {len(rows)} rows, {t_end:g} s -> {t_end + RAMP_S:g} s')


if __name__ == '__main__':
    main()
