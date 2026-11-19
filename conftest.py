"""pytest path bootstrap: make the stdlib-only core importable on the host.

trajectory.py is the stdlib-only core: it lives inside the ROS package (one source,
no copy) and imports no ROS, so plain ``python3 -m pytest`` works on the host.
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.join(_HERE, 'ros2_ws', 'src', 'crazyflies_traj'),
           _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)
