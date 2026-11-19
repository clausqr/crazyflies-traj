# AGENTS.md

ROS 2 (Jazzy) player that flies `x,y,z,t` CSV trajectories on Crazyflie drones.
See `README.md` for usage.

- Core logic is pure stdlib Python in `ros2_ws/src/crazyflies_traj/crazyflies_traj/`
  (`trajectory`, `sampler`, `plan`, `player_core`, `cli`); only `traj_player_node.py`
  and the launch files import ROS. Keep it that way.
- Tests: `python3 -m pytest` (Python 3.10+). Docker (optional): `./rr build|run|join|stop`.
- Try things with `python3 -m crazyflies_traj.cli check FILE.csv` (offline) or
  `ros2 run crazyflies_traj traj_player` (dry run). Never set `fly:=true`, and run
  `fleet.launch.py` only with `fly:=false`.
- Dry run must stay the default; never loosen the flight limits to make a file pass.
- This code commands real aircraft: no takeoff, landing, arming or e-stop code here.
- Plain ASCII punctuation. Branch from `develop`.
