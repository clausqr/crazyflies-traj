"""yaw: path (the nose follows the path tangent) and yaw: fixed unchanged."""

import hashlib
import math
import os

import pytest

from crazyflies_traj.cli import main
from crazyflies_traj.plan import check_offline, make_sampler
from crazyflies_traj.sampler import setpoint_table, YawSpec
from crazyflies_traj.trajectory import Limits, load_csv, parse_csv, TrajError

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CIRCLE = os.path.join(ROOT, 'trajectories', 'fleet', 'circle_1m_0.csv')
SQUARE = os.path.join(ROOT, 'trajectories', 'square_1m.csv')
PATH = YawSpec(yaw='path')
RATE = 50.0


def yaw_deg(sampler, t):
    return math.degrees(sampler.at(t).yaw)


def test_square_heading_with_the_hold_rule_at_the_corners():
    # pchip stops at every corner (per-axis monotone, zero slope at a turn), so the
    # heading is held through the stop and the turn is slewed (here at 360 deg/s).
    sq = parse_csv('x,y,z,t\n0,0,0.5,0\n1,0,0.5,2\n1,1,0.5,4\n0,1,0.5,6\n0,0,0.5,8\n')
    lim = Limits(yaw_rate_max_dps=360.0)
    s, demand = make_sampler(sq, 'pchip', (0, 0, 0), PATH, RATE, lim)
    assert demand == 0.0                         # straight sides: the path itself never turns
    for t, want in ((1.0, 0.0), (3.0, 90.0), (5.0, 180.0), (7.0, 270.0)):
        assert yaw_deg(s, t) == pytest.approx(want, abs=1e-6), t
    for t, held in ((2.0, 0.0), (4.0, 90.0), (6.0, 180.0)):
        assert yaw_deg(s, t) == pytest.approx(held, abs=1e-6), t    # the stop holds
    assert yaw_deg(s, 0.0) == pytest.approx(0.0)                    # before moving: the first
    assert yaw_deg(s, 99.0) == pytest.approx(270.0)                 # after: the last
    table = setpoint_table(s, RATE)
    assert max(abs(math.degrees(p.yaw_rate)) for p in table) <= 360.0 + 1e-6


@pytest.mark.parametrize('speedup, rate_dps', [(1.0, 6.0), (4.0, 24.0)])
def test_circle_turns_a_full_lap_continuously(speedup, rate_dps):
    traj = load_csv(CIRCLE, speedup=speedup)
    s, demand = make_sampler(traj, 'linear', (0, 0, 0), PATH, RATE, Limits())
    assert math.degrees(demand) == pytest.approx(rate_dps, rel=0.05)
    table = setpoint_table(s, RATE)
    yaws = [math.degrees(p.yaw) for p in table]
    # The tangent at (0.5, 0) is +Y (90 deg). The 4 s ramps start and end below
    # yaw_min_speed, so the first and last moving headings are held there: at 1x
    # (cruise 0.052 m/s) that is about 9 deg into each ramp, at 4x under 1 deg.
    assert 90.0 <= yaws[0] <= 100.0
    assert 340.0 <= yaws[-1] - yaws[0] <= 360.0                 # one lap, unwrapped
    assert all(b >= a for a, b in zip(yaws, yaws[1:]))           # never back, never a jump
    assert max(math.degrees(p.yaw_rate) for p in table) == pytest.approx(rate_dps, rel=0.05)


# sha256 of the --setpoints table printed by develop e87f0c6 (before yaw existed)
GOLDEN = {
    ('--interp', 'linear'): '3e55ad82425d3acf4a3f0f151bd50817c1096460095e370f3b181b715651e77e',
    ('--interp', 'pchip', '--acc-ff'):
        '38b0b79966ca03438794f3d8aec0cbbf531470f25c783972c2bd3c2110523dc6',
}


@pytest.mark.parametrize('opts', sorted(GOLDEN))
def test_fixed_mode_setpoint_table_is_byte_for_byte_unchanged(opts, capsys):
    assert main(['check', SQUARE, '--setpoints', *opts]) == 0
    lines = capsys.readouterr().out.splitlines(keepends=True)
    start = next(i for i, ln in enumerate(lines) if ln.startswith('#     t'))
    end = next(i for i, ln in enumerate(lines) if ln.startswith('ACCEPTED'))
    assert hashlib.sha256(''.join(lines[start:end]).encode()).hexdigest() == GOLDEN[opts]


def test_absurd_speedup_is_refused_as_yaw_rate(capsys):
    lim = Limits(a_max=20.0)                     # only the yaw can refuse
    traj = load_csv(CIRCLE, speedup=16.0)        # 96 deg/s
    assert check_offline(traj, lim, 'absolute', 'linear') == 0.0     # fixed: no yaw demand
    with pytest.raises(TrajError) as exc:
        check_offline(traj, lim, 'absolute', 'linear', PATH, RATE)
    assert exc.value.code == 'yaw_rate'
    rc = main(['check', CIRCLE, '--speedup', '16', '--a-max', '20', '--yaw', 'path'])
    err = capsys.readouterr().err
    assert rc == 1 and err.startswith('REFUSED yaw_rate:')


def test_check_prints_the_yaw_columns_and_the_demand(capsys):
    assert main(['check', CIRCLE, '--yaw', 'path', '--speedup', '4', '--setpoints']) == 0
    out = capsys.readouterr().out
    assert 'yaw           path, max rate 24.1 deg/s (yaw_rate_max_dps 90)' in out
    assert '      yaw  yawrate' in out
