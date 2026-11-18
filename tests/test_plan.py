"""Frames, binding to the world, and the dry-run summary."""

import pytest

from crazyflies_traj.plan import (bind, check_offline, format_setpoints, format_summary,
                                  frame_offset, summarize)
from crazyflies_traj.sampler import Sampler, setpoint_table
from crazyflies_traj.trajectory import Limits, parse_csv, TrajError

LINE = 'x,y,z,t\n0.2,0.1,0.5,0\n0.6,0.1,0.5,2\n0.6,0.5,0.7,4\n'


def test_absolute_offset_is_only_z0():
    tr = parse_csv(LINE)
    assert frame_offset(tr, 'absolute', 0.063) == (0.0, 0.0, 0.063)
    plan = bind(tr, Limits(), 'absolute', 'linear', 0.063, anchor_xy=(9.0, 9.0))
    assert plan.first == pytest.approx((0.2, 0.1, 0.563))
    assert plan.last == pytest.approx((0.6, 0.5, 0.763))


def test_relative_anchors_the_first_row_at_the_airborne_xy():
    tr = parse_csv(LINE)
    plan = bind(tr, Limits(), 'relative', 'linear', 0.05, anchor_xy=(-1.0, 0.3))
    assert plan.first == pytest.approx((-1.0, 0.3, 0.55))
    assert plan.last == pytest.approx((-0.6, 0.7, 0.75))
    with pytest.raises(TrajError):
        frame_offset(tr, 'relative', 0.05)          # no anchor
    with pytest.raises(TrajError):
        frame_offset(tr, 'body', 0.05)


def test_relative_geofence_rechecked_at_the_anchor():
    tr = parse_csv(LINE)
    check_offline(tr, Limits(), 'relative', 'linear')
    bind(tr, Limits(), 'relative', 'linear', 0.05, anchor_xy=(1.3, 0.0))       # +0.4 -> 1.7
    with pytest.raises(TrajError) as ei:
        bind(tr, Limits(), 'relative', 'linear', 0.05, anchor_xy=(1.4, 0.0))   # +0.4 -> 1.8
    assert ei.value.code == 'geofence'


def test_relative_pchip_rechecked_at_the_anchor():
    tr = parse_csv(LINE)
    bind(tr, Limits(), 'relative', 'pchip', 0.05, anchor_xy=(1.3, 0.0))
    with pytest.raises(TrajError):
        bind(tr, Limits(), 'relative', 'pchip', 0.05, anchor_xy=(1.4, 0.0))


def test_z_is_pad_relative_in_both_frames():
    tr = parse_csv(LINE)
    for frame in ('absolute', 'relative'):
        plan = bind(tr, Limits(), frame, 'linear', 0.10, anchor_xy=(0.2, 0.1))
        assert plan.sampler.at(0.0).pos[2] == pytest.approx(0.60)


def test_summary_numbers_square():
    tr = parse_csv('x,y,z,t\n0,0,0.5,0\n1,0,0.5,4\n1,1,0.5,8\n0,1,0.5,12\n0,0,0.5,16\n')
    sm = summarize(tr, 'linear')
    assert sm['duration_s'] == 16.0
    assert sm['path_length_m'] == pytest.approx(4.0)
    assert sm['max_speed_mps'] == pytest.approx(0.25)
    assert sm['max_accel_mps2'] == pytest.approx(0.125)
    assert sm['bbox_min'] == (0.0, 0.0, 0.5) and sm['bbox_max'] == (1.0, 1.0, 0.5)
    lines = format_summary(tr, 'linear', 'absolute', 50.0)
    assert any('801 at 50 Hz' in ln for ln in lines)
    table = setpoint_table(Sampler(tr, 'linear'), 50.0)
    out = format_setpoints(table, acc_ff=False)
    assert len(out) == 802 and out[1].split()[:4] == ['0.000', '0.000', '0.000', '0.500']
    assert len(format_setpoints(table, acc_ff=True)[1].split()) == 10


def test_speedup_divides_every_t_and_keeps_the_points():
    tr = parse_csv(LINE)
    fast = parse_csv(LINE, speedup=2.0)
    assert fast.points == tr.points and fast.speedup == 2.0
    assert fast.times == tuple(t / 2.0 for t in tr.times)
    assert fast.duration == pytest.approx(tr.duration / 2.0)


@pytest.mark.parametrize('bad', [0.0, -1.0, float('nan'), float('inf')])
def test_bad_speedup_is_refused(bad):
    with pytest.raises(TrajError) as e:
        parse_csv(LINE, speedup=bad)
    assert e.value.code == 'bad_speedup'
