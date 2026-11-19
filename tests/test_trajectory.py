"""trajectory.py: CSV parsing, linear interpolation, yaw."""

import math

import pytest

from crazyflies_traj.trajectory import parse_csv

TWO = 'x,y,z,t\n0,0,1,0\n2,0,1,2\n'


def test_header_any_order_with_comments_and_blanks():
    tr = parse_csv('# c\n\n T , Z,Y ,x\n0,1,2,3\n\n1,4,5,6\n')
    assert tr.points == [(3, 2, 1), (6, 5, 4)]
    assert tr.times == [0, 1]


def test_headerless_is_xyzt():
    assert parse_csv('1,2,3,0\n4,5,6,1\n').points == [(1, 2, 3), (4, 5, 6)]


def test_time_shifted_to_zero_and_speedup():
    tr = parse_csv('x,y,z,t\n0,0,0,10\n1,0,0,14\n', speedup=2.0)
    assert tr.times == [0, 2] and tr.duration == 2


@pytest.mark.parametrize('text,kw', [
    ('0,0,1\n1,0,1\n', {}),
    ('x,y,z,t\n0,0,a,0\n1,0,1,1\n', {}),
    ('x,y,z,t\n0,0,nan,0\n1,0,1,1\n', {}),
    ('x,y,z,t\n0,0,1,0\n', {}),
    ('x,y,z,t\n0,0,1,0\n1,0,1,0\n', {}),
    ('x,y,z,s\n0,0,1,0\n1,0,1,1\n', {}),
    (TWO, {'yaw': 'spin'}),
    (TWO, {'speedup': 0}),
])
def test_bad_input_raises(text, kw):
    with pytest.raises(ValueError):
        parse_csv(text, **kw)


def test_at_waypoints_and_midpoint():
    tr = parse_csv('x,y,z,t\n0,0,1,0\n2,0,1,2\n2,4,1,4\n')
    assert tr.at(2.0).pos == (2, 0, 1)
    mid = tr.at(1.0)
    assert mid.pos == pytest.approx((1, 0, 1)) and mid.vel == pytest.approx((1, 0, 0))


def test_at_clamps_with_zero_velocity_and_acc():
    tr = parse_csv(TWO)
    assert tr.at(-5).pos == (0, 0, 1) and tr.at(-5).vel == (0, 0, 0)
    assert tr.at(9).pos == (2, 0, 1) and tr.at(9).vel == (0, 0, 0)
    assert tr.at(1).acc == (0, 0, 0)


def test_yaw_fixed_is_radians():
    assert parse_csv(TWO, yaw_deg=90).at(1).yaw == pytest.approx(math.pi / 2)


def test_yaw_path_square_unwrapped_and_pause_holds():
    sq = 'x,y,z,t\n0,0,1,0\n1,0,1,1\n1,0,1,2\n1,1,1,3\n0,1,1,4\n0,0,1,5\n'
    tr = parse_csv(sq, yaw='path')
    degs = [math.degrees(tr.at(t).yaw) for t in (0.5, 1.5, 2.5, 3.5, 4.5)]
    assert degs == pytest.approx([0, 0, 90, 180, 270])
