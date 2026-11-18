"""
``traj``: offline trajectory check, no ROS (standard library only).

    python3 -m crazyflies_traj.cli check FILE.csv [--frame absolute|relative]
        [--interp linear|pchip] [--rate 50] [--z0 Z] [--setpoints]
        [--yaw fixed|path] [--yaw-fixed-deg 0] [--yaw-min-speed 0.05]
        [--no-header] [--normalize-t] [--x-min ..] [--v-max ..] ...

Validates the CSV against every limit that does not need the vehicle, prints
the summary (duration, path length, max speed, max accel, bounding box; with
``--yaw path`` the yaw rate the path demands) and, with ``--setpoints``, every
setpoint the player would stream (``--yaw path`` adds the yaw columns). Exit codes:
0 accepted, 1 refused (one ``REFUSED <code>: <why>`` line on stderr), 2 usage.
It never talks to a ROS graph, so it is safe anywhere, anytime.
"""

import argparse
from dataclasses import fields
import sys

from crazyflies_traj.plan import (check_offline, FRAMES, format_setpoints, format_summary,
                                  make_sampler)
from crazyflies_traj.sampler import INTERPOLATIONS, setpoint_table, YAW_MODES, YawSpec
from crazyflies_traj.trajectory import Limits, load_csv, TrajError

DEFAULT_RATE_HZ = 50.0


def add_limit_args(ap):
    """Add one ``--<limit>`` option per :class:`Limits` field (dashes for underscores)."""
    for f in fields(Limits):
        ap.add_argument('--' + f.name.replace('_', '-'), type=float, default=f.default,
                        dest=f.name, help=f'default {f.default:g}')


def limits_from(ns):
    """Build :class:`Limits` from an argparse namespace or a dict."""
    get = ns.get if isinstance(ns, dict) else (lambda k, d=None: getattr(ns, k, d))
    return Limits(**{f.name: float(get(f.name, f.default)) for f in fields(Limits)})


def yaw_from(ns):
    """Build :class:`YawSpec` from an argparse namespace (defaults for what is missing)."""
    return YawSpec(**{f.name: getattr(ns, f.name, f.default) for f in fields(YawSpec)})


def cmd_check(args, out=None, err=None):
    """Validate one CSV offline; print the summary (and setpoints)."""
    out = out or sys.stdout
    err = err or sys.stderr
    limits = limits_from(args)
    yaw = yaw_from(args)
    try:
        traj = load_csv(args.file, header=not args.no_header, normalize_t=args.normalize_t,
                        speedup=args.speedup)
        demand = check_offline(traj, limits, args.frame, args.interp, yaw, args.rate)
    except TrajError as exc:
        print(f'REFUSED {exc.code}: {exc.msg}', file=err)
        return 1
    z0 = args.z0 if args.z0 is not None else 0.0
    offset = (0.0, 0.0, z0)
    z_label = f'world, z0 {z0:g}' if args.z0 is not None else 'above pad'
    if args.frame == 'relative':
        z_label += '; x, y relative to the first row, anchored at trigger'
    yaw_line = (f'path, max rate {demand:.1f} deg/s (yaw_rate_max_dps '
                f'{limits.yaw_rate_max_dps:g})' if yaw.yaw == 'path' else None)
    for line in format_summary(traj, args.interp, args.frame, args.rate, offset, z_label,
                               yaw_line):
        print(line, file=out)
    if traj.speedup != 1.0:
        print(f'  speedup       x{traj.speedup:g} (the file as written lasts '
              f'{traj.duration * traj.speedup:.3f} s)', file=out)
    if args.setpoints:
        sampler = make_sampler(traj, args.interp, offset, yaw, args.rate, limits)[0]
        table = setpoint_table(sampler, args.rate)
        for line in format_setpoints(table, args.acc_ff, yaw.yaw == 'path'):
            print(line, file=out)
    print('ACCEPTED (offline checks; airborne, start-continuity and anchored geofence are '
          'checked by traj_player at trigger time)', file=out)
    return 0


def main(argv=None):
    """Parse and dispatch."""
    ap = argparse.ArgumentParser(prog='traj', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    c = sub.add_parser('check', help='validate a CSV offline and print the summary')
    c.add_argument('file')
    c.add_argument('--frame', choices=FRAMES, default='absolute')
    c.add_argument('--interp', choices=INTERPOLATIONS, default='linear')
    c.add_argument('--rate', type=float, default=DEFAULT_RATE_HZ, help='setpoint rate (Hz)')
    c.add_argument('--z0', type=float, default=None,
                   help='pad z0 (m) to print world z; default: z stays above-pad')
    c.add_argument('--setpoints', action='store_true', help='print every setpoint')
    c.add_argument('--acc-ff', action='store_true', help='include the acceleration columns')
    c.add_argument('--no-header', action='store_true',
                   help='the file has no header row (columns x, y, z, t)')
    c.add_argument('--normalize-t', action='store_true',
                   help='shift t so the first row is t=0')
    c.add_argument('--speedup', type=float, default=1.0,
                   help='divide every t by this: 2 flies the file in half the time (the '
                        'speed and acceleration limits see the faster motion); default 1')
    c.add_argument('--yaw', choices=YAW_MODES, default='fixed',
                   help='fixed: yaw-fixed-deg throughout; path: the heading of the path')
    c.add_argument('--yaw-fixed-deg', type=float, default=0.0, help='default 0')
    c.add_argument('--yaw-min-speed', type=float, default=0.05,
                   help='m/s; below it yaw path holds the last heading (default 0.05)')
    add_limit_args(c)
    args = ap.parse_args(argv)
    return cmd_check(args)


if __name__ == '__main__':
    sys.exit(main())
