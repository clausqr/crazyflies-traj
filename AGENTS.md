# AGENTS.md

ROS 2 (Jazzy) player that flies `x,y,z,t` CSV trajectories on Crazyflie drones.
See `README.md` for usage.

- Core logic is pure stdlib Python in `ros2_ws/src/crazyflies_traj/crazyflies_traj/`
  (`trajectory`, `sampler`, `plan`, `player_core`, `cli`); only `traj_player_node.py`
  and the launch files import ROS. Keep it that way.
- Tests: `python3 -m pytest` (Python 3.10+). Docker: `./rr build && ./rr rehearse`.
- Try things with `./rr traj check FILE.csv` (offline) or `./rr traj run FILE.csv`
  (dry run). Do not add `--fly` or run `rr traj fleet` without `--dry-run`.
- Dry run must stay the default; never loosen the flight limits to make a file pass.
- This code commands real aircraft: no takeoff, landing, arming or e-stop code here.
- Plain ASCII punctuation. Branch from `develop`.
