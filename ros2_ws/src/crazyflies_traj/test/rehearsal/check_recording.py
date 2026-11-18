#!/usr/bin/env python3
"""
Judge one rehearsal scenario from the mock plant's recording (standard library only).

    check_recording.py SCENARIO RECORD.jsonl PLAYER_RC PLAYER.log

The recording is JSON lines from mock_plant.py (plus ``sigint`` / ``intruder``
lines appended by run_rehearsal.sh and intruder.py), every line carrying
``mono`` (CLOCK_MONOTONIC, shared by all processes in the container). Prints
PASS or one FAIL line per violated expectation; exit 0 on PASS, 1 on FAIL.
The expectations per scenario are data (SPECS), so tests/test_check_recording.py
can feed seeded-bad recordings and witness every check going RED.
"""

import json
import statistics
import sys

from crazyflies_traj.player_core import Gates

TRACK_TOL = Gates().track_tol   # the player's own default: one source

RATE_HZ = 50.0
STABLE_S = 1.0
HOVER = (0.0, 0.0, 0.55)        # mock pad (0, 0) at z0 0.05, hover 0.5 above

SPECS = {
    # dry run: the gates run in shadow to DONE, nothing is ever published and
    # no publisher is ever created on any cmd topic
    'dry_run': dict(rc=0, max_cmds=0, max_pubs=0, log=['DRY RUN', 'DONE']),
    # the flight: nothing before airborne + stable, the exact plan, then hold
    'fly_short_line': dict(rc=0, min_cmds=330, max_cmds=400, after_airborne=True,
                           first_pos=HOVER, last_pos=HOVER, max_pubs=1, kind='full_state',
                           rate=RATE_HZ, log=['FLYING', 'HOLD', 'DONE', 'released']),
    'fly_position_cmd': dict(rc=0, min_cmds=330, max_cmds=400, after_airborne=True,
                             first_pos=HOVER, kind='position', max_pubs=1, rate=RATE_HZ),
    'fly_relative': dict(rc=0, min_cmds=330, max_cmds=400, after_airborne=True,
                         first_pos=(0.5, -0.5, 0.55), last_pos=(0.5, -0.5, 0.55), max_pubs=1),
    # a first setpoint 1.41 m from the hover: approached at 0.30 m/s peak (about
    # 7.1 s, first command = the hover point), then the 2 s line and the hold
    'fly_approach': dict(rc=0, min_cmds=500, max_cmds=700, after_airborne=True,
                         first_pos=HOVER, last_pos=(1.2, 1.0, 0.55), max_pubs=1,
                         log=['APPROACH', 'FLYING', 'HOLD', 'DONE']),
    # in-flight aborts: the stream stops within one status period + slack
    'abort_lock': dict(rc=1, min_cmds=20, stop_after='fault', stop_within=0.6,
                       log=['ABORT', 'IS_LOCKED', 'released']),
    'abort_tumble': dict(rc=1, min_cmds=20, stop_after='fault', stop_within=0.6,
                         log=['ABORT', 'IS_TUMBLED']),
    'abort_odom_stale': dict(rc=1, min_cmds=20, stop_after='fault', stop_within=0.5,
                             log=['ABORT', 'odom stale']),
    # measured from the first command whose plant error exceeds track_tol (the
    # fault onset is not the abort trigger: the drift takes seconds to get there)
    'abort_tracking': dict(rc=1, min_cmds=20, stop_after='track_cross', stop_within=0.1,
                           log=['ABORT', 'tracking error']),
    'abort_sigint': dict(rc=1, min_cmds=20, stop_after='sigint', stop_within=0.2,
                         log=['ABORT', 'SIGINT']),
    'abort_intruder': dict(rc=1, min_cmds=20, stop_after='intruder', stop_within=1.0,
                           log=['ABORT', 'single command authority']),
    # refusals: nothing ever published
    'refuse_intruder': dict(rc=1, max_cmds=0, log=['single command authority']),
    'refuse_never_airborne': dict(rc=1, max_cmds=0, log=['not airborne']),
    'refuse_bad_csv': dict(rc=1, max_cmds=0, max_pubs=0, log=['REFUSING TO FLY', 'speed']),
}


def load(path):
    """Read the JSON-lines recording."""
    with open(path, encoding='utf-8') as f:
        return [json.loads(line) for line in f if line.strip()]


def _close(a, b, tol=0.005):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def _err(cmd):
    """|plant position - setpoint| at the moment the mock received this command."""
    return sum((a - b) ** 2 for a, b in zip(cmd['at'], cmd['pos'])) ** 0.5


def judge(scenario, events, rc, log):
    """Return the list of failures (empty = PASS)."""
    spec = SPECS[scenario]
    fails = []
    cmds = [e for e in events if e['ev'] == 'cmd']
    n = len(cmds)
    if rc != spec['rc']:
        fails.append(f'player exit code {rc}, expected {spec["rc"]}')
    if 'max_cmds' in spec and n > spec['max_cmds']:
        fails.append(f'{n} commands received, at most {spec["max_cmds"]} allowed')
    if 'min_cmds' in spec and n < spec['min_cmds']:
        fails.append(f'{n} commands received, at least {spec["min_cmds"]} expected')
    if 'max_pubs' in spec:
        seen = [e.get('n', 0) for e in events if e['ev'] == 'publishers']
        seen += [e.get('max_publishers', 0) for e in events if e['ev'] == 'end']
        if max(seen, default=0) > spec['max_pubs']:
            fails.append(f'{max(seen)} publisher(s) seen on the cmd topics, at most '
                         f'{spec["max_pubs"]} allowed')
    if spec.get('after_airborne') and cmds:
        air = [e for e in events if e['ev'] == 'airborne']
        if not air:
            fails.append('commands received but the mock never went airborne')
        else:
            t_air = air[0]['mono']
            early = [c for c in cmds if c['mono'] < t_air + STABLE_S - 0.05]
            if early:
                fails.append(f'{len(early)} command(s) before airborne + {STABLE_S} s stable '
                             f'(first at {early[0]["mono"] - t_air:+.3f} s)')
    if 'first_pos' in spec and cmds and not _close(cmds[0]['pos'], spec['first_pos']):
        fails.append(f'first setpoint {cmds[0]["pos"]}, expected {spec["first_pos"]}')
    if 'last_pos' in spec and cmds and not _close(cmds[-1]['pos'], spec['last_pos']):
        fails.append(f'last setpoint {cmds[-1]["pos"]}, expected {spec["last_pos"]}')
    if 'kind' in spec and any(c.get('kind') != spec['kind'] for c in cmds):
        fails.append(f'a command was not {spec["kind"]}')
    if 'rate' in spec and len(cmds) > 10:
        dts = [b['mono'] - a['mono'] for a, b in zip(cmds, cmds[1:])]
        med = statistics.median(dts)
        if not 0.8 / spec['rate'] <= med <= 1.25 / spec['rate']:
            fails.append(f'median command period {med * 1000:.1f} ms, expected about '
                         f'{1000 / spec["rate"]:.0f} ms')
    if 'stop_after' in spec:
        if spec['stop_after'] == 'track_cross':
            # a player that aborts on the crossing streams no command beyond it
            # (the plant is sampled a tick later), so no crossing command is
            # fine; the fault must still be in the recording
            if not any(e['ev'] == 'fault' for e in events):
                fails.append('no fault event in the recording')
            marks = [c for c in cmds if _err(c) > TRACK_TOL]
            required = False
        else:
            marks = [e for e in events if e['ev'] == spec['stop_after']
                     or (e['ev'] == 'fault' and spec['stop_after'] == 'fault')]
            required = True
        if not marks:
            if required:
                fails.append(f'no {spec["stop_after"]} event in the recording')
        else:
            t_mark = marks[0]['mono']
            late = [c for c in cmds if c['mono'] > t_mark + spec['stop_within']]
            if late:
                fails.append(f'{len(late)} command(s) more than {spec["stop_within"]} s after '
                             f'{spec["stop_after"]} (last {late[-1]["mono"] - t_mark:+.3f} s)')
    for needle in spec.get('log', []):
        if needle not in log:
            fails.append(f'player log lacks {needle!r}')
    return fails


def main(argv):
    scenario, record, rc, logf = argv[1:5]
    with open(logf, encoding='utf-8', errors='replace') as f:
        log = f.read()
    fails = judge(scenario, load(record), int(rc), log)
    if fails:
        for msg in fails:
            print(f'FAIL {scenario}: {msg}')
        return 1
    n = sum(1 for e in load(record) if e['ev'] == 'cmd')
    print(f'PASS {scenario}: {n} command(s), player rc {rc}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
