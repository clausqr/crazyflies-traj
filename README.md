# crazyflies-traj: CSV trajectory player for Crazyflie swarms (ROS 2)

Plays a CSV trajectory (`x, y, z, t`) on one or several Bitcraze Crazyflies
from a ROS 2 Jazzy node. It checks the file against a flight envelope, then
streams setpoints once the drone is airborne and stable. Yaw is a parameter, not
a CSV column: `yaw:=fixed` (default, 0 deg) or `yaw:=path` (nose follows the
path). It needs a motion-capture groundstation (`crazyflies-gs`, not in this
repo; your instructor runs it). The **operator** is whoever runs the
groundstation and the flight.

## Quickstart

In a ROS 2 Jazzy shell with the package built (`cd ros2_ws && colcon build &&
source install/setup.bash`), from the repo root:

```bash
# check a file offline (no ROS graph needed)
python3 -m crazyflies_traj.cli check trajectories/square_1m.csv --setpoints --z0 0.063

# run it (publishes nothing until you add -p fly:=true)
ros2 run crazyflies_traj traj_player --ros-args -p csv_path:=$PWD/trajectories/square_1m.csv -p yaw:=path

# three-drone fleet from trajectories/fleet.yaml
ros2 launch crazyflies_traj fleet.launch.py config:=$PWD/trajectories/fleet.yaml
```

## CSV format

```csv
x,y,z,t
0.0000,0.0000,0.5000,0.000
1.0000,0.0000,0.5000,4.000
1.0000,1.0000,0.5000,8.000
```

- Header names `x, y, z, t` (any order); meters and seconds; `#` lines and
  blank lines are ignored; `nan`/`inf` are refused.
- `t` starts at 0 and strictly increases; at least 2 rows.
- `x, y` are in the motion-capture world frame (`absolute`, default) or offsets
  from the airborne position (`relative`). `z` is height above the pad.
- Yaw is not a column: `yaw:=fixed` (default) or `yaw:=path` (nose follows the path).
- Samples are in `trajectories/`; `trajectories/bad/` has one file per refusal code.

## Limits (`cli check --<name>` flags and player parameters)

| Limit | Default |
|---|---|
| x, y box | +-1.75 m around the origin |
| z (above pad) | 0.20 to 2.00 m |
| speed / acceleration | 2.5 m/s / 4.0 m/s^2 |
| duration | 120 s |
| start tolerance | 0.15 m (farther starts are approached first) |

A violating file is refused with `REFUSED <code>: <why>` and exit code 1; the
node then never creates a publisher. Codes: `bad_header`, `bad_row`,
`not_numeric`, `non_finite`, `too_few_rows`, `duplicate_row`, `non_monotone_t`,
`t_not_zero`, `too_long`, `geofence`, `below_floor`, `speed`, `accel`,
`yaw_rate`, `start_far`.

## Fleet

`trajectories/fleet.yaml` is a ROS 2 params file with one entry per vehicle:

```yaml
/cf1/traj_player:
  ros__parameters:
    csv_path: trajectories/fleet/circle_1m_0.csv
    yaw: path
/cf2/traj_player:
  ros__parameters:
    csv_path: trajectories/fleet/circle_1m_120.csv
```

The shipped file flies a three-drone circle (r 0.5 m, cf1/cf2/cf3 at 0, 120 and
240 degrees). **`fleet.launch.py` flies by default; pass `fly:=false` to only watch.**

## Safety

- **Dry run is the default** for `traj_player` (`fly:=false`). Flying needs
  `fly:=true`.
- The player never takes off, lands, or arms. The operator does that.
- The player publishes nothing until the vehicle is airborne, stable and fresh
  mocap and odometry are present. It aborts (and stops publishing) on stale
  data, large tracking error, another command publisher, or fatal supervisor
  flags. State machine:
  `WAIT_AIRBORNE -> ARMED_READY -> APPROACH -> READY -> FLYING -> HOLD -> DONE`,
  any state to `ABORT`.
- If the player exits while the drone is airborne, **land immediately**
  (`/cf1/land`, then `/cf1/notify_setpoints_stop`).

## License

Apache License 2.0, see [LICENSE](LICENSE).
