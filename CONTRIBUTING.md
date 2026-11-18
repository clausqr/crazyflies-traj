# Contributing: tests and guards

## Three layers

| Layer | Command | Needs | What it proves |
|---|---|---|---|
| Host unit tests | `python3 -m pytest` | host Python 3.10+ with pytest, nothing else | CSV parsing and every refusal, sampler math, frames, the player state machine (every gate and abort), the `traj` CLI, the rr harness guards (`run.bash --print`, no Docker), repo guards, the rehearsal judge |
| Mutation witness | `python3 tools/mutation_witness.py` | same | every guard's test goes RED when that guard is broken (exit 0 = all caught) |
| Offline rehearsal | `./rr build && ./rr rehearse` | Docker | the real `traj_player` against a mock plant in a throwaway container, domain 133, loopback DDS |

The stdlib-only core lives inside the ROS package
(`ros2_ws/src/crazyflies_traj/crazyflies_traj/{trajectory,sampler,plan,player_core,cli}.py`);
`conftest.py` puts it on the path, so there is one source and no copy. Keep
those modules free of ROS and third-party imports: the host suite must run with
plain `python3 -m pytest`. Only `traj_player_node.py` and `test/rehearsal/*`
(except `check_recording.py`) import ROS.

## House rules

- **Every guard ships a seeded-violation test**, witnessed failing: add the RED
  input to the tests AND a row to `tools/mutation_witness.py` that disables the
  guard, then run it. A guard never seen failing is not a guard.
- **Dry run first for the single-vehicle player.** `fly` defaults to false in
  the node and `traj_player.launch.py`, and the first player example in every doc
  must be a dry run; `tests/test_repo_guards.py` enforces all three. The fleet
  launch (`fleet.launch.py`, `rr traj fleet`) flies by default, by design
  (`fly:=false` / `--dry-run` for the shadow run).
- **No crutches.** A surprising threshold or sign upstream (plant, firmware,
  crazyflies-gs) is reported (README "Upstream observations"), not absorbed.
- **Never command hardware from a test.** The rehearsal refuses ROS domains 0
  and 66 and any non-loopback DDS profile; the image has no USB. Nothing here
  talks to the groundstation.
- English, no em-dashes (guarded), no tool-specific config files.
- Gitflow: branch from `develop`, merge back to `develop`; `main` only moves on
  a release.

## Adding a sample trajectory

Add it to `trajectories/generate.py` (or commit a CSV that passes
`./rr traj check`), keep it inside the default limits, start and end at rest at
the hover point (the flown Lissajous files end on the floor margin instead;
`tests/test_samples.py` names them). A deliberately bad file goes in `trajectories/bad/` named
`<refusal code>[__variant].csv`; `tests/test_samples.py` picks it up.
