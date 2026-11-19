# crazyflies-traj

A ROS 2 Jazzy node that plays a trajectory on a Crazyflie. It reads a CSV of
x, y, z, t, interpolates it linearly and publishes FullState setpoints on
`/<ns>/cmd_full_state` at `rate_hz`.

It starts publishing as soon as it starts (once a subscriber is matched and the
pad height is known). So take off first and hover, then start the node, and
land right after: it holds the last setpoint for `hold_s` seconds and then stops
publishing. It never takes off, lands or arms.

## Quickstart

From the repo root, in a ROS 2 Jazzy shell with the package built:

```
cd ros2_ws && colcon build && source install/setup.bash && cd ..
python3 -m crazyflies_traj.trajectory trajectories/square_1m.csv --yaw path      # print setpoints, no ROS
ros2 run crazyflies_traj traj_player --ros-args -p csv_path:=$PWD/trajectories/square_1m.csv -p yaw:=path
ros2 launch crazyflies_traj fleet.launch.py config:=$PWD/trajectories/fleet.yaml
```

## CSV format

```
# comment
x,y,z,t
0.0,0.0,0.5,0.0
1.0,0.0,0.5,4.0
```

- The header line is optional, `#` lines are comments.
- x, y in the world frame (m); z is the height above the pad (m); t in seconds.
- t is shifted to start at 0 and must be strictly increasing.
- nan and inf are rejected.

## Parameters

| name | default | meaning |
|---|---|---|
| csv_path | | trajectory file |
| ns | cf1 | vehicle namespace |
| rate_hz | 50 | setpoint rate |
| yaw | fixed | `fixed` or `path` (nose along the path) |
| yaw_deg | 0 | yaw for `fixed` |
| speedup | 1 | play faster (2 = half the time) |
| hold_s | 3 | seconds to hold the last setpoint |
| pad_z0 | nan | pad height; nan uses the latched `/<ns>/pad_z0` topic |

## Fleet

`fleet.launch.py` starts one node per top-level key of the yaml:

```
/cf1/traj_player:
  ros__parameters:
    csv_path: trajectories/fleet/circle_1m_0.csv
    speedup: 8.0
    yaw: path
```

`trajectories/fleet.yaml` flies three drones on one circle, 120 degrees apart.

## License

Apache License 2.0, see [LICENSE](LICENSE).
