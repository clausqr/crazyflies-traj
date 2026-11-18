"""`traj check`: exit codes and output (the offline half of the dry run)."""

import io
import os

from crazyflies_traj.cli import main

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SQUARE = os.path.join(ROOT, 'trajectories', 'square_1m.csv')
BAD_SPEED = os.path.join(ROOT, 'trajectories', 'bad', 'speed.csv')


def run(argv, capsys):
    rc = main(argv)
    out = capsys.readouterr()
    return rc, out.out, out.err


def test_accepts_and_summarizes(capsys):
    rc, out, _ = run(['check', SQUARE], capsys)
    assert rc == 0
    for needle in ('duration      16.000 s', 'path length   4.000 m', 'max speed     0.250',
                   'max accel     0.125', 'bbox x        [0.000, 1.000]', '801 at 50 Hz',
                   'ACCEPTED'):
        assert needle in out, needle


def test_setpoints_with_z0(capsys):
    rc, out, _ = run(['check', SQUARE, '--setpoints', '--z0', '0.063', '--rate', '10'], capsys)
    assert rc == 0
    rows = [ln for ln in out.splitlines() if ln.startswith('  ') and ln.split()[0][0].isdigit()
            and len(ln.split()) == 7]
    assert len(rows) == 161
    assert rows[0].split()[1:4] == ['0.000', '0.000', '0.563']


def test_refusal_exit_1_and_one_line(capsys):
    rc, out, err = run(['check', BAD_SPEED], capsys)
    assert rc == 1 and 'ACCEPTED' not in out
    assert err.startswith('REFUSED speed:') and len(err.strip().splitlines()) == 1


def test_limits_are_flags(capsys):
    rc, _, err = run(['check', SQUARE, '--x-max', '0.5'], capsys)
    assert rc == 1 and 'REFUSED geofence' in err
    rc, _, err = run(['check', SQUARE, '--v-max', '0.2'], capsys)
    assert rc == 1 and 'REFUSED speed' in err


def test_usage_error_exit_2(capsys):
    try:
        main(['check'])
    except SystemExit as exc:
        assert exc.code == 2
    else:
        raise AssertionError('argparse did not exit')
