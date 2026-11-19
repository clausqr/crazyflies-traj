#!/usr/bin/env python3
"""
Regenerate the sample trajectories in this directory (standard library only).

    python3 trajectories/generate.py

All three start and end at rest at (0, 0, 0.5): x, y in the Motive world frame
with the pad under the origin, z 0.5 m above the pad z0.
Curved paths ramp their speed up and down with a cosine blend so the start from
hover and the stop into the hold are gentle.
"""

import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
Z = 0.5


def _f(v):
    s = f'{v:.4f}'
    return '0.0000' if s == '-0.0000' else s


def write(name, rows, note):
    with open(os.path.join(HERE, name), 'w', encoding='utf-8') as f:
        f.write(f'# {note}\n')
        f.write('x,y,z,t\n')
        for x, y, z, t in rows:
            f.write(f'{_f(x)},{_f(y)},{_f(z)},{t:.3f}\n')


def ramped(total_len, v, t_ramp, dt):
    """Arc length s(t) for cruise speed v with cosine ramps up and down."""
    # Ramp distance: integral of v * (1 - cos(pi t / T)) / 2 over T = v T / 2.
    d_ramp = v * t_ramp / 2.0
    cruise = total_len - 2.0 * d_ramp
    if cruise < 0:
        raise ValueError('path too short for the ramps')
    t_cruise = cruise / v
    t_end = 2.0 * t_ramp + t_cruise
    out = []
    n = int(math.ceil(t_end / dt - 1e-9))
    dt = t_end / n                      # an even grid that lands exactly on t_end
    for k in range(n + 1):
        t = k * dt
        if t < t_ramp:
            s = v * (t / 2.0 - t_ramp / (2.0 * math.pi) * math.sin(math.pi * t / t_ramp))
        elif t < t_ramp + t_cruise:
            s = d_ramp + v * (t - t_ramp)
        else:
            u = t - t_ramp - t_cruise
            s = d_ramp + cruise + v * (u / 2.0 + t_ramp / (2.0 * math.pi)
                                       * math.sin(math.pi * u / t_ramp))
        out.append((t, min(s, total_len)))
    out[-1] = (t_end, total_len)
    return out


def square():
    # 1 m square, 0.25 m/s, straight legs (linear interpolation's home turf).
    pts = [(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)]
    rows = [(x, y, Z, 4.0 * i) for i, (x, y) in enumerate(pts)]
    write('square_1m.csv', rows, '1 m square at 0.5 m above the pad, 0.25 m/s, 16 s')


def circle():
    # r = 0.5 m, centre (0.5, 0): starts at the origin, 0.2 m/s cruise.
    r, cx = 0.5, 0.5
    length = 2.0 * math.pi * r
    rows = []
    for t, s in ramped(length, 0.2, 3.0, 0.25):
        th = math.pi + s / r
        rows.append((cx + r * math.cos(th), r * math.sin(th), Z, t))
    write('circle_r05.csv', rows, 'circle r 0.5 m around (0.5, 0) at 0.5 m above the pad, '
                                  '0.2 m/s cruise, cosine ramps')


def figure8():
    # Lemniscate of Gerono, x = A sin(th), y = (A/2) sin(2 th): through the origin.
    a = 0.8
    ths = [2.0 * math.pi * k / 4000 for k in range(4001)]
    pts = [(a * math.sin(th), 0.5 * a * math.sin(2.0 * th)) for th in ths]
    cum = [0.0]
    for p, q in zip(pts, pts[1:]):
        cum.append(cum[-1] + math.hypot(q[0] - p[0], q[1] - p[1]))
    length = cum[-1]
    rows = []
    j = 0
    for t, s in ramped(length, 0.2, 3.0, 0.25):
        while j < len(cum) - 2 and cum[j + 1] < s:
            j += 1
        seg = cum[j + 1] - cum[j]
        u = 0.0 if seg == 0 else (s - cum[j]) / seg
        x = pts[j][0] + u * (pts[j + 1][0] - pts[j][0])
        y = pts[j][1] + u * (pts[j + 1][1] - pts[j][1])
        rows.append((x, y, Z, t))
    rows[-1] = (0.0, 0.0, Z, rows[-1][3])
    write('figure8.csv', rows, 'gentle figure-8 (Gerono, 1.6 x 0.8 m) at 0.5 m above the pad, '
                               '0.2 m/s cruise, cosine ramps')


if __name__ == '__main__':
    square()
    circle()
    figure8()
