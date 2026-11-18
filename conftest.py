"""pytest path bootstrap: make the stdlib-only core importable on the host.

The pure modules live inside the ROS package (one source, no copy):
ros2_ws/src/crazyflies_traj/crazyflies_traj/{trajectory,sampler,plan,player_core,cli}.py
and the rehearsal checker in ros2_ws/src/crazyflies_traj/test/rehearsal/.
None of them imports ROS, so plain ``python3 -m pytest`` works on the host.
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.join(_HERE, 'ros2_ws', 'src', 'crazyflies_traj'),
           os.path.join(_HERE, 'ros2_ws', 'src', 'crazyflies_traj', 'test', 'rehearsal'),
           _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)
