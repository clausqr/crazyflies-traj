# crazyflies-traj: CSV trajectory player for Crazyflie swarms (ROS 2)

Fly a CSV trajectory (`x, y, z, t`) on one or several **Bitcraze Crazyflie**
nano quadcopters from a **ROS 2** node, with offline validation, rest-to-rest
limits and a dry run by default. Works with a motion-capture setup (OptiTrack
Motive) through the **crazyflies-gs** groundstation plant: `traj_player` reads
and validates the CSV and, once the vehicle is **already airborne and stable**,
streams it as setpoints on the plant's command seam (`/cf1/cmd_full_state`).
It is a client of the plant, like any other plant client, not part of crazyflies-gs.

Two terms used throughout this README:

- **groundstation**: the separate crazyflies-gs motion-capture plant. It is not
  in this repo; your lab or instructor provides it.
- **operator**: the person running the groundstation and the flight.

**No groundstation? You can still run most of this.** Use `python3 -m pytest`
(host unit tests), `./rr traj check FILE.csv` (offline validation) and
`./rr build && ./rr rehearse` (the real player against a mock plant in a
container). Flying needs the groundstation and its operator.

Typical uses: multi-drone formation flight (a ready-made three-drone circle is
included), scripted indoor flight paths from a spreadsheet, and repeatable
trajectory-tracking experiments on a Crazyflie 2.x fleet.

## Features

- **Offline check first:** `rr traj check` validates a CSV with no ROS graph and
  refuses anything outside the flight envelope (x, y within 1.75 m, z 0.2 to
  2.0 m, speed 2.5 m/s, acceleration 4 m/s^2, up to 120 s).
- **Dry run by default:** a shadow run against the live graph prints every
  setpoint it would send and publishes nothing.
- **Multi-drone fleets:** one `fleet.yaml` launches one player per Crazyflie,
  with the yaw following the path (`yaw: path`).
- **Spreadsheet to flight:** edit an `.xlsx`, export one CSV per drone; a
  generator and a three-drone circle example are included.
- **Offline rehearsal** against a mock plant in a throwaway container, so the
  node is exercised without hardware.
- **Lean and reproducible:** a small Docker image (ROS 2 Jazzy base) driven by
  one `./rr` script, plus a host test suite that needs only Python.

Scope of v1: it never takes off, lands, arms or calls
`notify_setpoints_stop`. The operator owns takeoff and landing. **It is a dry run by default** and publishes nothing
unless `fly:=true` / `--fly` is given explicitly.

- Stdlib-only core (CSV, limits, sampler, state machine), tested on the host
  with plain `python3 -m pytest`.
- Lean rr container (ros-base + two message packages) for the node and an
  offline rehearsal against a mock plant.
- The two spherical Lissajous samples have been flown on hardware. Every other
  hardware claim below is either read from vendored code (file cited) or listed
  under "To witness at the lab".

## Contents

1. [Quickstart (dry run first)](#quickstart-dry-run-first)
2. [CSV format](#csv-format)
3. [Frames](#frames)
4. [Limits and the refusal table](#limits-and-the-refusal-table)
5. [Setpoints: interpolation, rate, command topic](#setpoints-interpolation-rate-command-topic)
6. [Multi-vehicle: the fleet config](#multi-vehicle-the-fleet-config)
7. [Safety model](#safety-model)
8. [The rr harness](#the-rr-harness)
9. [Offline rehearsal](#offline-rehearsal)
10. [Handing the trajectory to the groundstation](#handing-the-trajectory-to-the-groundstation)
11. [Upstream observations](#upstream-observations)
12. [Layout, branches](#layout-branches)

## Quickstart (dry run first)

```bash
./rr build                                     # lean image + colcon build (once)
python3 -m pytest                              # host unit tests, no ROS needed

./rr traj check square_1m.csv                  # offline: summary, refusals, no ROS graph
./rr traj check square_1m.csv --setpoints --z0 0.063   # every setpoint, world z
./rr traj run square_1m.csv                    # DRY RUN against the live graph (shadow)
./rr traj fleet fleet.yaml --dry-run           # one player per vehicle in the yaml; without --dry-run it FLIES
```

`rr traj run` without `--fly` prints the summary and every setpoint it would
send, then runs the airborne gates in **shadow** against the graph and logs what
it would do (`[DRY RUN] ARMED_READY ...`, `FLYING ...`). It never creates a
command publisher, so it cannot command anything and never appears in the
plant's single-command-authority publisher count.

Flying is the operator's decision and only follows the full sequence in
[Handing the trajectory to the groundstation](#handing-the-trajectory-to-the-groundstation):

```bash
./rr traj run square_1m.csv --name traj --fly   # only after the dry run
```

The same through ROS directly (inside the container):

```bash
ros2 launch crazyflies_traj traj_player.launch.py csv_path:=/traj/square_1m.csv            # dry run
ros2 run crazyflies_traj traj_player --ros-args -p csv_path:=/traj/square_1m.csv -p fly:=true
```

Prefer `ros2 run` for scripted use: its exit code is the player's (0 DONE,
1 refused or aborted, 2 bad parameters); `ros2 launch` does not propagate it.

## CSV format

```csv
# 1 m square at 0.5 m above the pad, 0.25 m/s, 16 s
x,y,z,t
0.0000,0.0000,0.5000,0.000
1.0000,0.0000,0.5000,4.000
1.0000,1.0000,0.5000,8.000
0.0000,1.0000,0.5000,12.000
0.0000,0.0000,0.5000,16.000
```

| Rule | Detail |
|---|---|
| Header | one row naming exactly `x, y, z, t` (any order, case and blanks ignored). A headerless file needs `--no-header` / `header:=false`, and its order is then fixed to `x, y, z, t`. |
| Units | meters and seconds. |
| `x, y` | Motive world (`absolute`, default) or offsets anchored at trigger (`relative`), see [Frames](#frames). |
| `z` | **height above the pad z0** in both frames (the pad-relative convention of the plant's clients). World z = `z + z0`. |
| `t` | starts at exactly 0 and strictly increases. A nonzero start is refused unless `--normalize-t` / `normalize_t:=true` shifts every `t` by the first. `--speedup N` / `speedup:=N` divides every `t` by N (2 = the file in half the time): the speed and acceleration limits then check the faster motion, so a file that was fine at x1 can be refused at x2. |
| Values | finite numbers only (`nan`, `inf`, unit suffixes refused). |
| Rows | at least 2; a row identical to the previous one is refused (`duplicate_row`); the same position at a later `t` is a hover pause and is fine. |
| Comments | blank lines and lines starting with `#` are ignored. |
| Yaw | not a column. `yaw:=fixed` (default) commands `yaw_fixed_deg` (0) throughout; `yaw:=path` commands the heading of the sampled velocity, `atan2(vy, vx)`, so the nose follows the path tangent (see [Setpoints](#setpoints-interpolation-rate-command-topic)). |

Samples in `trajectories/` (regenerate with `python3 trajectories/generate.py`;
a test fails if the files drift from the generator). All start and end at rest
at `(0, 0, 0.5)`, the pad under the Motive origin, except the two Lissajous
files, which end at `(0, 0, 0.2)` on the floor margin:

| File | Path | Duration | Max speed | Max accel (linear / pchip) |
|---|---|---|---|---|
| `square_1m.csv` | 1 m square | 16.0 s | 0.25 m/s | 0.125 / 0.375 m/s^2 |
| `circle_r05.csv` | circle r 0.5 m around (0.5, 0), cosine speed ramps | 18.7 s | 0.20 m/s | 0.107 / 0.271 m/s^2 |
| `figure8.csv` | Gerono figure-8, 1.6 x 0.8 m, cosine ramps | 27.4 s | 0.20 m/s | 0.228 / 0.580 m/s^2 |
| `spherical_lissajous_slow.csv` | spherical Lissajous, 2.0 x 2.0 x 1.5 m, flown | 65.0 s | 0.96 m/s | 0.913 m/s^2 / refused |
| `spherical_lissajous_fast.csv` | the same path at twice the speed, flown | 32.5 s | 1.91 m/s | 3.65 m/s^2 / refused |

The two Lissajous files are not generated: they are committed as flown. They
are linear-only: 4 s lead-in rows followed by 20 ms rows make the pchip curve's
acceleration spike far above `a_max` at the junction, so `interp:=pchip`
refuses them (`accel`).

`trajectories/bad/<code>[__variant].csv` are deliberately bad files, one per
refusal code (the test suite requires each to be refused with the code in its
name). `trajectories/rehearsal/` holds the short lines the rehearsal flies.

## Frames

| Mode | x, y | z | Geofence checked |
|---|---|---|---|
| `absolute` (**default**) | Motive world, Z-up: the frame of `/<ns>/odom` and `/poses` | above pad z0 | completely, offline, before the vehicle is involved |
| `relative` | offsets; the first row is placed at the vehicle's airborne x, y at trigger time | above pad z0 | extent offline; the anchored path at trigger time |

**Why `absolute` is the default:** what the dry run prints is exactly what
flies (with `--z0` the table shows world z too), and the geofence is proven
before takeoff. With `relative` the world path only exists at trigger time, so
the reviewable artifact is weaker; its geofence is re-checked when anchoring
and the flight is refused if the anchored path leaves the box. In both modes
`z` is never re-anchored to wherever the vehicle happens to hover: a hover at
the wrong height is caught by the start-continuity check instead, so the
floor margin always means the same thing.

With `yaw:=fixed` (default) yaw is held at `yaw_fixed_deg` (0).
With `yaw:=path` the approach turns the nose from the measured `/<ns>/odom` yaw to
the first setpoint's heading at `yaw_rate_max_dps`, then the nose follows the path
(no yaw step at takeover).

## Limits and the refusal table

Each default is a `traj check --<name>` flag and a `traj_player` parameter of
the same name (underscores). The geofence, `z_max`, `v_max`, `a_max` and the
abort's `track_tol` were sized to the envelope of the first spherical Lissajous
flights (the earlier, tighter values were x, y +-1.5 m, `z_max` 1.50 m, `v_max`
0.50 m/s, `a_max` 1.0 m/s^2, `track_tol` 0.30 m).

| Limit | Default | Why |
|---|---|---|
| `x_min, x_max, y_min, y_max` | -1.75, 1.75 m | a 3.5 m box around the Motive origin; the flown Lissajous reached y -1.64 m (`relative`) |
| `z_floor` | 0.20 m above pad | floor margin: ground effect and a topple seen on an early flight |
| `z_max` | 2.00 m above pad | the flown Lissajous reached 1.72 m |
| `v_max` | 2.50 m/s | the fast Lissajous peaks at 1.91 m/s |
| `a_max` | 4.0 m/s^2 | includes the start from hover and the stop into the hold; the fast Lissajous reads 3.65 m/s^2 |
| `start_tol` | 0.15 m | first setpoint vs the airborne position (3D); farther starts are approached first |
| `approach_v` | 0.30 m/s | peak speed of the approach (a `traj_player` parameter, not a `traj check` flag) |
| `max_duration` | 120 s | one battery-safe leg |
| `yaw_rate_max_dps` | 90 deg/s | `yaw:=path`: the yaw slew rate, and the most the path may demand (more is refused, `yaw_rate`) |
| `yaw`, `yaw_fixed_deg`, `yaw_min_speed` | `fixed`, 0 deg, 0.05 m/s | yaw mode (a `traj check --yaw` flag and a player parameter); below `yaw_min_speed` (horizontal) `path` holds the last heading |

Speed is checked per linear segment; acceleration as the finite-difference
velocity change at each waypoint (rest to rest, over the mean adjacent segment
duration; a 10-degree-sampled circle reads v^2/r within 1 percent). With
`interp:=pchip` the smooth curve's own peak speed and acceleration are checked
too (they can exceed the polyline's).

| Code | Refused when | Where |
|---|---|---|
| `unreadable` | file cannot be read | offline |
| `bad_header` | header is not exactly `x, y, z, t` (hint when it looks numeric) | offline |
| `bad_row` | a row does not have 4 fields | offline |
| `not_numeric` | a field is not a number | offline |
| `non_finite` | `nan` / `inf` | offline |
| `too_few_rows` | fewer than 2 data rows | offline |
| `duplicate_row` | a row repeats the previous one | offline |
| `non_monotone_t` | `t` does not strictly increase | offline |
| `t_not_zero` | first `t` is not 0 (without `normalize_t`) | offline |
| `bad_speedup` | `speedup` is not a finite number > 0 | offline |
| `too_long` | duration above `max_duration` | offline |
| `geofence` | a waypoint (or the pchip curve) leaves the box or exceeds `z_max`; relative: the extent fits no anchor, or the anchored path leaves the box | offline / trigger |
| `below_floor` | height above pad under `z_floor` | offline |
| `speed` | a segment (or the pchip curve) above `v_max` | offline |
| `accel` | a waypoint (or the pchip curve) above `a_max` | offline |
| `yaw_rate` | `yaw:=path`: the heading turns faster than `yaw_rate_max_dps` between two moving setpoints (at the given `speedup` and `rate_hz`) | offline |
| `start_far` | first setpoint more than `start_tol` from the airborne position and no approach planner (the node always has one: it approaches instead) | trigger |
| `bad_limits`, `bad_frame`, `bad_interp`, `bad_rate`, `bad_gates` | nonsense configuration | startup |

Offline refusals exit `traj check` with 1 and one `REFUSED <code>: <why>`
line; the node logs `REFUSING TO FLY: <code>: ...` and exits 1 without
creating a publisher. Trigger-time refusals (`start_far`, anchored `geofence`,
another command publisher, fatal supervisor bits) happen before authority is
taken: nothing is ever published.

## Setpoints: interpolation, rate, command topic

- **Interpolation.** `linear` (default): straight segments; velocity
  feed-forward = segment velocity; acceleration 0. `pchip`: per-axis monotone
  cubic Hermite (Fritsch-Carlson), zero velocity at both ends. Monotone per axis
  means the curve never leaves the range of its two waypoints on any axis, so it
  cannot overshoot an axis-aligned geofence the waypoints are inside (tested on
  the box face). Use `linear` for dense CSVs (the samples) and `pchip` for sparse
  waypoints.
- **Yaw.** `yaw:=path` takes the heading of the interpolated velocity on the
  setpoint grid: held below `yaw_min_speed`, unwrapped, slewed to
  `yaw_rate_max_dps`, linear in between; the yaw rate is the finite difference
  (FullState `twist.angular.z`, rad/s; `Position.yaw` carries degrees). A turn at a
  stop (pchip corners) is slewed in place; the HOLD keeps the last yaw.
- **Timing.** Setpoints are evaluated at the elapsed wall time since FLYING
  began (timer jitter does not accumulate); the dry-run table is the same
  function at `k / rate_hz`, ending exactly on the last waypoint. After the last
  point the player holds it (zero velocity) for `hold_s` (default 3 s).
- **Rate: 50 Hz default** (`rate_hz`, allowed 10 to 100). The plant has no
  rate requirement beyond the firmware watchdog (500 ms level-out, 2 s lock,
  `supervisor.c:58-59`): the cflib server forwards every message as one CRTP
  packet (`crazyflie_server.py` `_cmd_full_state_changed`). 50 Hz is 25x inside
  the watchdog and halves the uplink load of a 100 Hz stream on a
  radio that already carries 100 Hz odom + 100 Hz `ekf_quat` downlink; at the
  2.5 m/s speed cap a 20 ms step is 5 cm (1 cm at 0.5 m/s).
- **Command topic: `/<ns>/cmd_full_state`** (`crazyflie_interfaces/FullState`)
  by default, `command:=position` selects `/<ns>/cmd_position` (`Position`,
  position + yaw only). Why full state: velocity
  feed-forward is honored by the Mellinger controller and discarded by PID, and
  the groundstation pins `stabilizer.controller: 2` (Mellinger) on hardware,
  checked by the plant's readiness check. Under Mellinger a
  position-only command tracks a moving reference with a zero-velocity demand,
  i.e. it drags. Acceleration feed-forward stays off by default (`acc_ff`);
  it only means something for `pchip`.
- **QoS.** The server subscribes `cmd_*` with the rclpy default (reliable,
  volatile, keep last 10, `crazyflie_server.py:393-416`). The player's publisher
  is reliable, volatile, keep last 10: a best-effort publisher would not match a
  reliable subscriber. Subscriptions: `/<ns>/odom`, `/poses`, `/<ns>/status`
  best-effort (matches both reliable and best-effort publishers; `/poses` is
  best-effort on the rig); `/<ns>/pad_z0` reliable +
  transient_local, depth 1 (the latched contract, `pad_z0_node.py`).
- **Message fill.** `FullState`: `header.frame_id = world`, pose position,
  orientation = the setpoint yaw (identity for `yaw:=fixed`, 0 deg),
  `twist.linear` = velocity, `acc` = 0 unless `acc_ff`, `twist.angular.z` = the
  yaw rate (0 for `fixed`). Interface fields are guarded against the apt
  packages by `check_interfaces.py` (vs crazyswarm2 `f1e0995`).

## Multi-vehicle: the fleet config

`trajectories/fleet.yaml` maps each vehicle namespace to its own CSV and player
parameters. It is a standard ROS 2 params file, one entry per player node
`/<ns>/traj_player` (rcl's parser is strict: every top-level key is a node name
holding `ros__parameters`):

```yaml
/cf1/traj_player:
  ros__parameters:
    csv_path: /traj/fleet/circle_1m_0.csv
    yaw: path             # optional: the nose follows the circle's tangent
/cf2/traj_player:
  ros__parameters:
    csv_path: /traj/fleet/circle_1m_120.csv
    speedup: 1.0          # optional, any player parameter: 2.0 flies this file in half the time
    yaw: path
```

The shipped file flies the three-drone circle: r 0.5 m around the
origin, 1.0 m above the pad, cf1, cf2 and cf3 at 0, 120 and 240 deg of the same
circle so they stay 0.87 m from each other, 64 s each (`circle_1m_0.csv`,
`circle_1m_120.csv`, `circle_1m_240.csv`). A two-drone pair (`circle_1m_0.csv`
and `circle_1m_180.csv`, 1.0 m apart) and the larger three circles (r 1.41 m,
`circle_0/120/240.csv`) stay in `trajectories/fleet/`. The originals as given sit in
`fleet/src/`, and `tools/ramp_ends.py` writes the flown copies with the first
and last 4 s ramped from and to rest (as given they start at full speed in a
10 ms row, which the rest-to-rest check refuses as `accel`). Each is absolute,
so every player approaches its own start from wherever its vehicle hovers.

### Example: three-drone circle

1 m diameter (r 0.5 m), 30 deg/s, 120 deg between drones, 1.0 m above the pad, yaw along
the trajectory (`yaw: path`), 40 s (three laps plus 4 s cosine ramps from and to rest).
Files in `trajectories/examples/`:

- `circulo-3-drones.xlsx`: editable workbook, sheets `cf1`, `cf2`, `cf3` (columns `x,y,z,t`)
  and `notas` (the parameters). Save each sheet as CSV UTF-8 (comma, decimal point).
- `circulo-3-drones_cf1.csv`, `_cf2.csv`, `_cf3.csv`: what the player reads (it never reads xlsx).
- `circulo-3-drones_fleet.yaml`: one `/cfN/traj_player` per drone with `yaw: path`.

```
./rr traj fleet examples/circulo-3-drones_fleet.yaml --dry-run
```

`tools/make_circle_xlsx.py` writes all of them (stdlib only; `--radius`, `--omega-dps`,
`--laps`, `--z` change the circle) and `--xlsx-to-csv FILE` turns an edited workbook back
into CSVs.

Paths are as seen in the container (`/traj` = `trajectories/`). `csv_path` is
required; any other traj_player parameter is a per-vehicle override of the
node default. `fly` and `ns` are not taken from the file: the launch sets them
after the file's values (the namespace is the key, flight is the command line's).
The launch hands each node a dict, not the file, so a YAML int such as
`rate_hz: 50` is cast to the double the node declares (rclpy refuses an int for
a double parameter).

```bash
./rr traj fleet fleet.yaml --dry-run                           # shadow run, every vehicle
./rr traj fleet fleet.yaml --name fleet                     # FLIES every vehicle in the yaml
ros2 launch crazyflies_traj fleet.launch.py config:=/traj/fleet.yaml             # same, in the container
ros2 launch crazyflies_traj fleet.launch.py config:=/traj/fleet.yaml fly:=false  # dry run
```

**The fleet flies by default** (by design): `fleet.launch.py`
and `rr traj fleet` take authority as soon as each vehicle is airborne and
stable, unlike the single-vehicle `traj_player`, which stays a dry run until
`fly:=true`. The yaml itself can neither enable nor disable flight.

Vehicles are independent: each player has its own gates, its own
`/<ns>/traj_abort` and its own command authority on `/<ns>/cmd_*`; there is no
fleet abort (Ctrl-C stops them all). **They do start together.** The launch
hands every player the roster (`fleet` parameter); once a player is at its
first setpoint (after its approach) it enters READY, keeps streaming that point
and latches `/<ns>/traj_ready`; it starts its CSV when every roster vehicle
present on the graph has latched ready too (absent vehicles are skipped), or
after `sync_timeout` (30 s) on its own. All present players therefore leave
their starts on the same tick, within a message hop of each other. A vehicle named in the
file but absent from the graph (the deploy ladder flies cf1 first, then
cf1 + cf2, then all three) gets one `ABSENT` warning 2 s after start; its
player waits `wait_timeout` (120 s) and exits 1 on its own while the others
fly. `ros2 launch` does not
propagate the players' exit codes: read each node's `DONE` / `ABORT` lines.

## Safety model

```
WAIT_AIRBORNE -> ARMED_READY -> APPROACH -> READY -> FLYING -> HOLD -> DONE
      \______________\____________\___________\________\________\____> ABORT
```

**Nothing is published before FLYING, and the command publisher does not exist
before ARMED_READY.** The pure state machine is `player_core.py`; the node only
samples the graph and acts on its verdict.

### Airborne (and stable)

All of the following, continuously for `stable_s` = 1.0 s:

| Condition | Source | Default |
|---|---|---|
| pad z0 known | latched `/<ns>/pad_z0` or `pad_z0` parameter | - |
| status fresh | `/<ns>/status` (1 Hz on the rig) | age <= `status_timeout` 3.0 s |
| supervisor bits set | IS_ARMED (2), CAN_FLY (8), IS_FLYING (16) | - |
| supervisor bits clear | IS_TUMBLED (32), IS_LOCKED (64), IS_CRASHED (128) | - |
| odom fresh | `/<ns>/odom` (100 Hz) | age <= `odom_timeout` 0.25 s |
| height above pad | `odom.z - z0` | enter at `h_enter` 0.25 m, drop out below `h_exit` 0.15 m (hysteresis) |
| mocap fresh and anchored | `/poses` entry named `<ns>` | age <= 0.25 s, within `anchor_tol` 0.10 m of `/<ns>/odom` |
| stable | `|v_odom|` (norm; the hw twist is body-frame) | <= `v_stable` 0.30 m/s |

IS_FLYING is the firmware's "a motor above idle thrust within the last 2 s"
(`supervisor.c` `isFlyingCheck`), so it is set during an HL hover. IS_CRASHED
(0x80) is set by the firmware (`supervisor.c` `updateLogData`) but has no
constant in `Status.msg`; the player names it itself. `require_poses:=false`
drops the mocap condition (not recommended: a dead mocap leaves a
plausible-looking `/<ns>/odom`). No airborne within
`wait_timeout` (120 s) refuses.

### Taking authority (ARMED_READY)

At the tick the vehicle is airborne and stable: bind the plan to the world
(`relative`: anchor at the current x, y) and re-check the geofence;
start-continuity (`start_tol`: a farther first setpoint is not refused but planned
as an **approach**, a smooth move at `approach_v` peak inside `a_max` and the
geofence, streamed before the CSV with no gap in the stream); **no publisher from
any other node on any of the
five low-level command topics** the cflib server subscribes
(`cmd_full_state`, `cmd_position`, `cmd_vel_legacy`, `cmd_velocity_world`,
`cmd_hover`). Any failure refuses with nothing published. Then the player
creates its publisher and waits (at most `authority_timeout` 3 s) until the
plant's command gate (`cmd_gate`, or `cmd_gate_<ns>` when the plant flies several
vehicles) is subscribed and matched, then streams. The gate relays to the server's
`/<ns>/gated/cmd_*` only while the daemon's latched `/<ns>/cmd_mute` is false (opened
at the arm, closed before every land): a groundstation land beats the stream.

### Aborts (APPROACH, FLYING and HOLD)

The same tick destroys the publisher (never keeps streaming) on:
supervisor IS_LOCKED / IS_TUMBLED / IS_CRASHED; IS_FLYING cleared; status,
odom or poses stale; tracking error `|odom - setpoint|` above `track_tol`
(0.60 m); another publisher appearing on a command topic; SIGINT / SIGTERM; a
message on `/<ns>/traj_abort` (`std_msgs/Empty`). E-stop is detected as
IS_LOCKED on the next status message (the e-stop latches the supervisor); the
e-stop itself acts in firmware independently of the player.

### End behaviour, and the hazard the operator owns

After the last point the player holds it for `hold_s`, then destroys its
publisher and exits 0. **Stopping a low-level stream does not hand the vehicle
back to anything.** From the vendored firmware:

- While the player streams, each setpoint arrives at `COMMANDER_PRIORITY_CRTP`
  (2), above the high-level commander (1), and stops the HL planner
  (`commander.c` `commanderSetSetpoint` -> `crtpCommanderHighLevelStop`).
- When the stream stops, the last low-level setpoint ages: at 500 ms the
  supervisor levels out (keeps z, lets x, y drift), at 2 s it **locks the motors**
  (`supervisor.c:58-59`, latched until reboot). The vehicle falls from the hold
  height.
- An HL `land` issued now plans from the current state, but its setpoints are
  ignored while the stale priority 2 is held (`stabilizer.c:335`,
  `commanderSetSetpoint`: priority 1 < 2).
- `notify_setpoints_stop` relaxes the priority (`crtp_commander.c`
  `notifySetpointsStopDecoder` -> `commanderRelaxPriority`). If the HL planner is
  idle at that moment it returns a null setpoint: **motors off**
  (`crtp_commander_high_level.c:361-369`).

So the hand-back is `land` FIRST, then `notify_setpoints_stop`, both within
about 0.5 s of the stream stopping and strictly within 2 s. A cold
`ros2 service call` pays process start and DDS discovery (the reason
the groundstation keeps a pre-warmed e-stop client), so it is too slow to be the
hand-back. This is the main open design question (see the next
section): v1 does not land.

## The rr harness

Modelled on crazyflies-gs (`rr`, `scripts/`, `setup.env`, `Dockerfile`,
`ros2_packages.txt`) but lean: ros-base (digest-pinned) plus `ros-jazzy-crazyflie-interfaces` and
`ros-jazzy-motion-capture-tracking-interfaces`; about 0.9 GB against the plant
image's 4.7 GB. No cffirmware, no simulator, no server, no USB, no mocap client.

| Command | What |
|---|---|
| `./rr build` | build `crazyflies-traj:v0.2.0` (+ `:latest`), then colcon-build `ros2_ws` in a throwaway container (`--net=none`) |
| `./rr run [--name N] [--cmd '...' [--fg]] [--print]` | the container: `ros2_ws` at `/ros2_ws`, `trajectories/` read-only at `/traj`, the DDS profile dir read-only at `/ros2_ws/config`; `--print` shows the docker command only |
| `./rr join [--name N] [--cmd '...']` | a shell (or command) in a running container |
| `./rr stop [--name N]` | stop it (TERM: the player aborts and destroys its publisher) |
| `./rr traj check FILE [opts]` | offline check in a `--net=none` container |
| `./rr traj run FILE [--name N] [key:=value ...] [--fly]` | the player in the foreground (Ctrl-C = abort); dry run unless `--fly`; `fly:=` and `csv_path:=` are refused as key:=value |
| `./rr rehearse [scenario ...]` | the offline rehearsal (below) |

`setup.env` highlights: `RR_ROS_DOMAIN_ID` defaults to **66** (the lab plant's
domain) and is overridden per run for anything else (`./rr rehearse` defaults
to 133 and refuses 0 and 66). `RR_NET_MODE=lan` (default: the lab plant over the
rig LAN) or `loopback` (one box; `./rr rehearse` selects it itself). The interactive
shells of `rr run` and `rr join` print a MOTD with the domain, net mode and
profile in force plus the dry-run commands to start from, and have tab
completion for `ros2` (packages, launch files, executables, topics) and `colcon`.

**DDS profiles (`dds/`, owned by this repo).** `run.bash` mounts `dds/`
(`RR_DDS_DIR`) read-only at `/ros2_ws/config`: loopback uses the committed
`fastdds_loopback.xml` / `cyclonedds_loopback.xml`; lan (default) uses
`fastdds_client.xml` / `cyclonedds_client.xml`, which are git-ignored and come
from the groundstation's static `/dds/` page (`RR_DDS_URL`). The Fast DDS client
profile carries the marker `__OWN_IP__`; put this machine's IP in it (Cyclone
has no marker). From a bare clone:

```bash
curl -fu USER:PASSWORD -o dds/fastdds_client.xml --create-dirs http://GROUNDSTATION/dds/fastdds_client.xml
curl -fu USER:PASSWORD -o dds/cyclonedds_client.xml http://GROUNDSTATION/dds/cyclonedds_client.xml
sed -i "s/__OWN_IP__/$(hostname -I | awk '{print $1}')/g" dds/fastdds_client.xml
./rr build
./rr run
```

`GROUNDSTATION`, `USER` and `PASSWORD` are your site's values (ask your operator). Put them once in `rr.local` (copy `rr.local.example`; git-ignored): `RR_DDS_URL` and `RR_DDS_LOGIN` then fill in the curl line that a refused run prints. A missing profile refuses the run (with the curl line), and so does a Fast DDS
profile that still contains `__OWN_IP__` (with the sed line): no silent fallback
to an uncontained DDS on `--net=host`. There is no staleness check: re-download
and re-sed after a groundstation address change or a new IP on this machine.
`RR_USB_ENABLED=1` is refused.

## Offline rehearsal

`./rr rehearse` runs the real `traj_player` executable against
`test/rehearsal/mock_plant.py` (TEST ONLY: node `cmd_gate_cf1`, `/cf1/odom`,
`/poses`, `/cf1/status`, latched `/cf1/pad_z0`, a scripted takeoff at 2 s,
first-order following, the 2 s watchdog lock, fault injection) in a throwaway
`--rm` container on domain 133 over the loopback profile in `dds/`. Each
scenario's recording is judged by `check_recording.py`:

| Scenario | Witnesses |
|---|---|
| `dry_run` | nothing received, no publisher ever on a cmd topic, shadow reaches DONE |
| `fly_short_line` | no command before airborne + 1 s stable; first setpoint = hover point; about 375 commands at 50 Hz (4.5 s + 3 s hold); last = hold point; one publisher |
| `fly_position_cmd`, `fly_relative` | `cmd_position`; relative anchoring at (0.5, -0.5) |
| `abort_lock`, `abort_tumble`, `abort_odom_stale`, `abort_tracking` | stream stops within one status period (+ slack) of the fault |
| `abort_sigint`, `abort_intruder` | SIGINT; a second publisher appearing mid-flight |
| `fly_approach` | a first setpoint 1.41 m from the hover: the approach streams from the hover point, then the line |
| `refuse_intruder`, `refuse_never_airborne`, `refuse_bad_csv` | exit 1, zero commands |

Plus the interface drift check (and its seeded self-test) and a seeded
cross-check: real recordings judged against the wrong expectations must go RED.

## Handing the trajectory to the groundstation

The contract this client relies on, for the groundstation side to honor.
Nothing here edits crazyflies-gs.

**What the plant consumes** (cflib `crazyflie_server`, crazyswarm2 `f1e0995`):

| Item | Value |
|---|---|
| Topic | `/<ns>/cmd_full_state` (`cf1`), alt. `/<ns>/cmd_position` |
| Type | `crazyflie_interfaces/msg/FullState` (`header`, `pose`, `twist`, `acc`); `Position` (`x, y, z, yaw`) |
| QoS | `cmd_gate` subscription (relayed to the server's `/<ns>/gated/cmd_*` while `/<ns>/cmd_mute` is false): reliable, volatile, keep last 10; publisher must be reliable |
| Rate | no server requirement; firmware watchdog 500 ms level-out, 2 s lock; player 50 Hz |
| Frame | Motive world, Z-up, meters (`reference_frame: world`); heights above the latched `/<ns>/pad_z0` |
| Controller | Mellinger (`stabilizer.controller: 2`, checked by the plant's readiness check): position + velocity tracked |

**Single command authority.** The plant's readiness check allows at most one
publisher per `cmd_*` topic. The player adds
the run-time half: it refuses to take authority while any other node publishes
on any of the five `cmd_*` topics, and aborts if one appears. It must never run
beside any other command path.

**Preconditions:** the plant's readiness check passes, the vehicle
airborne and hovering under the HL commander (takeoff via the HL
`takeoff` service), the warm e-stop shell ready (`estop_client serve`), the recorder
up, the dry run reviewed.

**The flight sequence** (exact commands). `./rr` lines run from this repo on
the client machine; `ros2 ...` lines run in a ROS shell on the lab domain (e.g.
`./rr join --name traj`, or the plant container):

```bash
# 1. check the file offline
./rr traj check FILE.csv --setpoints --z0 "$(ros2 topic echo /cf1/pad_z0 --once \
    --qos-durability transient_local --qos-reliability reliable --field point.z)"

# 2. dry run: shadow against the live graph
./rr traj run FILE.csv --name traj

# 3. fly
#  a. gate: the plant's readiness check passes and no other command path is up
#  b. HL takeoff (onboard trajectory, no stream; height is ABSOLUTE = z0 + 0.5)
ros2 service call /cf1/takeoff crazyflie_interfaces/srv/Takeoff \
    "{group_mask: 0, height: <z0+0.5>, duration: {sec: 2, nanosec: 0}}"
#  c. the player (foreground; exits 0 DONE / 1 abort or refusal)
./rr traj run FILE.csv --name traj --fly
#  d. IMMEDIATELY on ANY exit while airborne, from a PRE-WARMED client
#     (not a cold `ros2 service call`): land FIRST, then notify
ros2 service call /cf1/land crazyflie_interfaces/srv/Land \
    "{group_mask: 0, height: <z0>, duration: {sec: 3, nanosec: 0}}"
ros2 service call /cf1/notify_setpoints_stop crazyflie_interfaces/srv/NotifySetpointsStop \
    "{group_mask: 0, remain_valid_millisecs: 100}"
#     backstop: props still spinning 1 s after touchdown ->
ros2 service call /cf1/arm crazyflie_interfaces/srv/Arm "{arm: false}"
```

Step d's timing is the open question: see "End behaviour" above. Options:
(a) a warm hand-back client on the groundstation (the `estop_client` pattern)
fired on the player's exit; (b) a v2 player that performs
`land` then `notify_setpoints_stop` itself on DONE and on non-fatal aborts
(stale mocap, tracking, intruder, SIGINT), which removes the gap entirely.

**To witness at the lab** (not verifiable offline):

1. The rig firmware (2026.08) has the vendored commander semantics above:
   stream priority 2 stops the HL planner; HL `land` after the stream stops and
   before `notify_setpoints_stop` lands smoothly; 500 ms level-out; 2 s lock.
2. Mellinger tracks `FullState` position + velocity on the real airframe at
   0.2 to 0.25 m/s (commanded vs achieved), and a 20 ms step at
   50 Hz causes no audible or visible stepping.
3. Latency from publish on the client machine to the radio over the LAN DDS profile, and
   radio headroom with 50 Hz uplink + 200 Hz telemetry.
4. `/cf1/status` rate and supervisor bits in an HL hover: IS_FLYING (16) set,
   IS_ARMED + CAN_FLY set (30 total expected), IS_CRASHED (128) behaviour.
5. The pad z0 contract on the rig: `/cf1/pad_z0` latched and matching mocap at
   rest; the player's `h_enter` 0.25 m relative to that z0.
6. `/poses` named `cf1` at about 120 Hz reaches the client box; `anchor_tol`
   0.10 m holds in a hover.
7. The command gate's node name as seen from the client machine: `cmd_gate` for one vehicle,
   `cmd_gate_<ns>` for several (the player waits for it to subscribe the command
   topic).
8. Hover yaw after HL takeoff is near 0 (`yaw:=fixed` commands yaw 0; `yaw:=path`
   turns from the measured yaw at `yaw_rate_max_dps`).
9. The geofence defaults against the real flight volume (the +-1.75 m
   box followed the flights, not a volume measurement).
10. A dry run (`rr traj run`, no `--fly`) against the live plant during a
    hover reaches `[DRY RUN] FLYING` with no other command path up.

## Upstream observations

Reported, not worked around:

1. **The plant's publisher-count check is narrower than its description.**
   The readiness check is described as "at most one publisher on every
   `/<ns>/cmd_*` topic", but its probe only inspects `cmd_full_state` and `cmd_position`; the server also subscribes
   `cmd_vel_legacy`, `cmd_velocity_world` and `cmd_hover`. The player checks
   all five itself.
2. **Stopping a stream is not a hand-back** (firmware analysis above). Leaving
   landing to the operator after the stream stops leaves a 0.5 s /
   2 s window no cold CLI meets; whoever owns landing needs a warm path.
3. **`Status.msg` lacks the firmware's newer supervisor bits** (0x80 crashed,
   0x100 HL active, ...), which the firmware does set.
4. **Shared piece worth extracting:** the rr `run.bash` DDS-profile selection
   selection is now needed by every client repo. This repo owns a `dds/` folder
   filled from the groundstation's static profile page; no sibling checkout.
5. The mocap stand-in had to publish a real twist: a player gate on `|v|` with
   a zero-twist `/<ns>/odom` (as a relay or replay that drops twist would give) passes
   during a climb. On the rig the plant's readiness check guards this.

## Layout, branches

```
rr, setup.env, Dockerfile, ros2_packages.txt   lean rr harness
scripts/        build, run, join, stop, traj, rehearse
trajectories/   samples, bad/ (one per refusal code), rehearsal/, generate.py
ros2_ws/src/crazyflies_traj/
  crazyflies_traj/  trajectory.py sampler.py plan.py player_core.py cli.py (stdlib only)
                    traj_player_node.py (rclpy)
  launch/           traj_player.launch.py
  test/rehearsal/   mock_plant.py intruder.py check_recording.py check_interfaces.py run_rehearsal.sh
tests/          host pytest suite
tools/          mutation_witness.py, modify_speed.py (fast Lissajous = slow with t halved)
```

Gitflow: `develop` is the default and integration line; `main` advances only
via release merges. See `CONTRIBUTING.md` for tests.

## License

Apache License 2.0, see [LICENSE](LICENSE).
