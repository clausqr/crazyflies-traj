"""
The fleet config: one traj_player per vehicle. Standard library only (no ROS, no yaml).

The file is a standard ROS 2 params file (README "Multi-vehicle: the fleet
config"), one entry per player node, ``/<ns>/traj_player``::

    /cf1/traj_player:
      ros__parameters:
        csv_path: /traj/square_1m.csv
    /cf2/traj_player:
      ros__parameters:
        csv_path: /traj/circle_r05.csv
        frame: relative

rcl's params-file parser is strict (every top-level key is a node name holding
``ros__parameters``), so the fleet is expressed in exactly that shape.
fleet.launch.py loads it (PyYAML is a launch-time dependency, not a core one),
calls :func:`vehicles` and hands each node its own dict. ``fly`` and ``ns`` are
set by the launch AFTER the file's values, so the file can never enable flight.
"""

from dataclasses import fields
import re

from crazyflies_traj.player_core import Gates
from crazyflies_traj.sampler import YawSpec
from crazyflies_traj.trajectory import Limits, TrajError

NODE_KEY = re.compile(r'^/([A-Za-z][A-Za-z0-9_]*)/traj_player$')

# Parameters the node declares as double. A YAML ``rate_hz: 50`` loads as an
# int, and rclpy refuses an int for a double parameter at declare time (the
# node dies before it prints anything useful), so these are cast here.
FLOAT_KEYS = ({'rate_hz', 'hold_s', 'pad_z0'}
              | {f.name for f in fields(Limits) if isinstance(f.default, float)}
              | {f.name for f in fields(Gates) if isinstance(f.default, float)}
              | {f.name for f in fields(YawSpec) if isinstance(f.default, float)})


def vehicles(doc):
    """Return [(ns, params)] from a loaded params-file dict, in file order."""
    out = []
    for key, val in (doc or {}).items():
        m = NODE_KEY.match(str(key))
        params = val.get('ros__parameters') if isinstance(val, dict) else None
        if not m or not isinstance(params, dict) or not params.get('csv_path'):
            raise TrajError('bad_fleet', f'{key!r}: expected a /<ns>/traj_player entry whose '
                            'ros__parameters hold csv_path')
        out.append((m.group(1), {k: float(v) if k in FLOAT_KEYS and isinstance(v, int)
                                 and not isinstance(v, bool) else v
                                 for k, v in params.items()}))
    return out
