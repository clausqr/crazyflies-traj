# AGENTS.md

- ROS 2 Jazzy player of x,y,z,t CSV trajectories on Crazyflies; see README.md.
- The core is the stdlib module `crazyflies_traj/trajectory.py`; only
  `traj_player_node.py` and `launch/` import ROS.
- Tests: `python3 -m pytest`.
- No takeoff, landing or arming code here.
- Plain ASCII punctuation.
- Branch from `develop`.
