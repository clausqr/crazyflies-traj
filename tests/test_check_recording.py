"""
The rehearsal judge on synthetic recordings: a clean recording passes, and each
seeded violation (early command, command in a dry run, command after an abort,
second publisher, wrong first setpoint, wrong rate, wrong exit code) goes RED.
"""

import check_recording as cr

HOVER = [0.0, 0.0, 0.55]


def flight(n=375, t_air=10.0, t_first=11.2, dt=0.02, pos=HOVER, kind='full_state'):
    ev = [{'ev': 'start', 'mono': 0.0},
          {'ev': 'airborne', 'mono': t_air, 'pos': HOVER},
          {'ev': 'publishers', 'mono': t_first - 0.1, 'n': 1}]
    ev += [{'ev': 'cmd', 'mono': t_first + k * dt, 'pos': list(pos), 'kind': kind}
           for k in range(n)]
    ev.append({'ev': 'end', 'mono': t_first + n * dt + 1, 'max_publishers': 1})
    return ev


GOOD_LOG = 'FLYING ... HOLD ... DONE ... released'


def test_clean_flight_passes():
    assert cr.judge('fly_short_line', flight(), 0, GOOD_LOG) == []


def test_command_before_airborne_is_red():
    fails = cr.judge('fly_short_line', flight(t_first=10.5), 0, GOOD_LOG)
    assert any('before airborne' in f for f in fails)


def test_command_in_a_dry_run_is_red():
    ev = [{'ev': 'start', 'mono': 0.0}, {'ev': 'cmd', 'mono': 5.0, 'pos': HOVER},
          {'ev': 'end', 'mono': 9.0, 'max_publishers': 0}]
    fails = cr.judge('dry_run', ev, 0, 'DRY RUN DONE')
    assert any('at most 0' in f for f in fails)


def test_publisher_in_a_dry_run_is_red():
    ev = [{'ev': 'start', 'mono': 0.0}, {'ev': 'publishers', 'mono': 1.0, 'n': 1},
          {'ev': 'end', 'mono': 9.0, 'max_publishers': 1}]
    fails = cr.judge('dry_run', ev, 0, 'DRY RUN DONE')
    assert any('publisher(s) seen' in f for f in fails)


def test_second_publisher_in_a_flight_is_red():
    ev = flight()
    ev.append({'ev': 'publishers', 'mono': 12.0, 'n': 2})
    fails = cr.judge('fly_short_line', ev, 0, GOOD_LOG)
    assert any('2 publisher(s)' in f for f in fails)


def test_streaming_after_the_fault_is_red():
    ev = flight(n=200)
    ev.append({'ev': 'fault', 'mono': 11.2 + 0.5, 'what': 'lock'})
    fails = cr.judge('abort_lock', ev, 1, 'ABORT IS_LOCKED released')
    assert any('after fault' in f for f in fails)
    ok = flight(n=40) + [{'ev': 'fault', 'mono': 11.2 + 0.5, 'what': 'lock'}]
    assert cr.judge('abort_lock', ok, 1, 'ABORT IS_LOCKED released') == []


def test_missing_fault_marker_is_red():
    fails = cr.judge('abort_sigint', flight(n=40), 1, 'ABORT SIGINT')
    assert any('no sigint event' in f for f in fails)


def test_wrong_exit_code_wrong_start_wrong_kind_wrong_rate_missing_log():
    assert any('exit code' in f for f in cr.judge('fly_short_line', flight(), 1, GOOD_LOG))
    assert any('first setpoint' in f for f in
               cr.judge('fly_short_line', flight(pos=[0.1, 0, 0.55]), 0, GOOD_LOG))
    assert any('not full_state' in f for f in
               cr.judge('fly_short_line', flight(kind='position'), 0, GOOD_LOG))
    assert any('command period' in f for f in
               cr.judge('fly_short_line', flight(dt=0.04), 0, GOOD_LOG))
    assert any('lacks' in f for f in cr.judge('fly_short_line', flight(), 0, 'FLYING'))


def test_refusals_must_publish_nothing():
    for name in ('refuse_intruder', 'refuse_never_airborne', 'refuse_bad_csv'):
        spec_log = ' '.join(cr.SPECS[name]['log'])
        assert cr.judge(name, [{'ev': 'end', 'mono': 1, 'max_publishers': 0}], 1, spec_log) == []
        assert cr.judge(name, flight(n=1), 1, spec_log)


def test_tracking_abort_is_timed_from_the_error_crossing_not_the_fault():
    tol = cr.TRACK_TOL
    t_fault, dt = 11.2 + 0.5, 0.02

    def drift(n_after_cross):
        ev = flight(n=0) + [{'ev': 'fault', 'mono': t_fault, 'what': 'drift'}]
        t_cross = t_fault + 3.0          # 0.1 m/s setpoint vs 0.3 m/s drift
        for i in range(-int(3.0 / dt), n_after_cross):
            err = tol + 0.002 * (i + 1) if i >= 0 else tol * 0.5
            ev.append({'ev': 'cmd', 'mono': t_cross + i * dt, 'pos': [0.0, 0.0, 0.55],
                       'at': [err, 0.0, 0.55], 'kind': 'full_state'})
        return ev

    log = 'ABORT tracking error'
    assert cr.judge('abort_tracking', drift(2), 1, log) == []      # stops one tick after
    assert cr.judge('abort_tracking', drift(0), 1, log) == []      # stops before the crossing
    fails = cr.judge('abort_tracking', drift(40), 1, log)          # streams 0.8 s past it
    assert any('after track_cross' in f for f in fails)
