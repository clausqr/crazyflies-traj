# crazyflies-traj: CSV trajectory player for Crazyflie swarms (ROS 2)

Fly a CSV trajectory (`x, y, z, t`) on one or several **Bitcraze Crazyflie**
quadcopters from a **ROS 2** (Jazzy) node. It validates the file offline,
refuses anything outside a flight envelope, and, once the vehicle is already
airborne and stable, streams the path as setpoints (`/cf1/cmd_full_state`) to a
motion-capture groundstation (the separate `crazyflies-gs` plant, not in this
repo; your lab or instructor provides it). The **operator** is the person
running the groundstation and the flight.

**No groundstation? Most of this still runs:**

```bash
python3 -m pytest                      # host unit tests, Python 3.10+ and pytest only
./rr traj check trajectories/square_1m.csv     # offline validation, no ROS
./rr build && ./rr rehearse            # the real player vs a mock plant (Docker)
```

## Quickstart (dry run first)

```bash
./rr build                                      # lean image + colcon build, once
./rr traj check square_1m.csv --setpoints --z0 0.063   # every setpoint, world z
./rr traj run square_1m.csv                     # DRY RUN against the live graph, publishes nothing
./rr traj fleet fleet.yaml --dry-run            # one player per vehicle; without --dry-run it FLIES
./rr traj run square_1m.csv --name traj --fly   # only after the dry run
```

Site values (groundstation URL and login) go in `rr.local`, copied from
`rr.local.example` (git-ignored; the operator gives you the values).

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

## Limits (all are `traj check --<name>` flags and player parameters)

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
    csv_path: /traj/fleet/circle_1m_0.csv
    yaw: path
/cf2/traj_player:
  ros__parameters:
    csv_path: /traj/fleet/circle_1m_120.csv
```

The shipped file flies a three-drone circle (r 0.5 m, cf1/cf2/cf3 at 0, 120 and
240 degrees). **`rr traj fleet` flies by default; pass `--dry-run` first.**

## Safety

- **Dry run is the default** for `rr traj run` / `traj_player` (`fly:=false`).
  Flying needs `--fly` / `fly:=true`.
- The player never takes off, lands, or arms. The operator does that.
- The player publishes nothing until the vehicle is airborne, stable and fresh
  mocap and odometry are present. It aborts (and stops publishing) on stale
  data, large tracking error, another command publisher, or fatal supervisor
  flags. State machine:
  `WAIT_AIRBORNE -> ARMED_READY -> APPROACH -> READY -> FLYING -> HOLD -> DONE`,
  any state to `ABORT`.
- If the player exits while the drone is airborne, **land immediately**
  (`/cf1/land`, then `/cf1/notify_setpoints_stop`).
- Offline rehearsal refuses ROS domains 0 and 66 and any non-loopback DDS
  profile, and the image has no USB, so tests can never command hardware.

## License

Apache License 2.0, see [LICENSE](LICENSE).
