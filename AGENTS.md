# AGENTS.md

Guidance for AI coding tools (and humans) working in this repository.
`README.md` is the full reference; this file is the short orientation.

## What this is

`crazyflies-traj` plays a CSV trajectory (`x, y, z, t`) on one or several
Bitcraze Crazyflie quadcopters from a ROS 2 (Jazzy) node. It validates the file
offline, refuses anything outside a flight envelope, and streams setpoints to a
motion-capture groundstation plant. It is a client of that plant; the
groundstation itself is not in this repository.

## Safety first

This code can command real aircraft. Keep these invariants:

- **Dry run is the default** for the single-vehicle player (`fly` is `false` in
  the node and in `traj_player.launch.py`). Do not change a default to make it
  fly. The first example in any doc must be a dry run.
- The fleet launch (`rr traj fleet`) flies unless `--dry-run` is given, so
  always pass `--dry-run` when trying it.
- Never add code that takes off, lands, arms, or sends emergency stops. That is
  out of scope for this player by design.
- Never run anything that publishes commands from a test. Tests and rehearsal
  are loopback only.
- Do not loosen the envelope limits (geofence, speed, acceleration, duration)
  to make a trajectory pass. Fix the trajectory instead.

## Layout

- `ros2_ws/src/crazyflies_traj/crazyflies_traj/`: the package.
  `trajectory.py`, `sampler.py`, `plan.py`, `player_core.py`, `cli.py` are
  **pure stdlib Python** (no ROS, no third-party imports). Only
  `traj_player_node.py` and `fleet.py`/launch files touch ROS.
- `tests/`: host unit tests. `conftest.py` puts the package on the path.
- `trajectories/`: sample CSVs, `generate.py` (regenerates the generated ones),
  `bad/` (one deliberately bad file per refusal code), `fleet.yaml` and
  `examples/` (a three-drone circle plus its spreadsheet).
- `tools/`: spreadsheet and CSV helpers, `mutation_witness.py`.
- `rr`, `scripts/`, `Dockerfile`, `setup.env`: Docker harness.
  `rr.local.example` shows the site settings; copy it to `rr.local`
  (git-ignored) and fill it in. Never commit `rr.local`.

## Commands

```bash
python3 -m pytest                        # host tests, needs only Python 3.10+ and pytest
python3 tools/mutation_witness.py        # every guard's test must go RED when the guard is broken
python3 trajectories/generate.py         # regenerate sample CSVs (a test fails on drift)
./rr traj check trajectories/circle_r05.csv   # offline validation, no ROS, no hardware
./rr traj run trajectories/circle_r05.csv     # DRY RUN against a live ROS graph (publishes nothing)
./rr build && ./rr rehearse              # needs Docker; real node vs a mock plant, loopback only
```

You can do most work with just `python3 -m pytest` and `./rr traj check`.
Flying needs the Crazyflie hardware, a motion-capture setup and the
groundstation, which students normally do not have outside the lab.

## Conventions

- Run `python3 -m pytest` before proposing a change; it must pass.
- Every guard ships a seeded-violation test: add the failing input to the tests
  and a row to `tools/mutation_witness.py`, and confirm it goes red.
- Keep the stdlib-only core free of ROS and third-party imports.
- No hacks that hide a problem in the plant or firmware; document it under
  "Upstream observations" in `README.md` instead.
- Plain ASCII punctuation: no em-dashes (a test enforces it).
- Add a new sample trajectory via `trajectories/generate.py`, or commit a CSV
  that passes `./rr traj check`. Add a bad example as
  `trajectories/bad/<refusal_code>[__variant].csv`.
- Branches: start from `develop`, open pull requests against `develop`.

See `CONTRIBUTING.md` for the details and `README.md` for the CSV format,
frames, limits and refusal codes.
