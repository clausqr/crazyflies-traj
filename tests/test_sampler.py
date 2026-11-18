"""Sampler: endpoints exact, continuity, derivative consistency, no overshoot, timing math."""

import math

import pytest

from crazyflies_traj.sampler import (check_sampled_limits, path_length, sample_times, Sampler,
                                     setpoint_table)
from crazyflies_traj.trajectory import Limits, parse_csv, TrajError

SQUARE = 'x,y,z,t\n0,0,0.5,0\n1,0,0.5,4\n1,1,0.5,8\n0,1,0.5,12\n0,0,0.5,16\n'
# Irregular spacing and a zig-zag with a pause: the hard case for overshoot.
ZIGZAG = ('x,y,z,t\n0,0,0.5,0\n0.3,0.1,0.6,1.5\n0.1,0.4,0.9,2.2\n0.1,0.4,0.9,3.0\n'
          '0.6,-0.2,0.4,5.5\n0.65,0.0,0.45,6.0\n0,0,0.5,9\n')


def close(a, b, tol=1e-12):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


@pytest.mark.parametrize('interp', ['linear', 'pchip'])
def test_endpoints_and_waypoints_exact(interp):
    tr = parse_csv(ZIGZAG)
    s = Sampler(tr, interp)
    for t, p in zip(tr.times, tr.points):
        assert s.at(t).pos == p, (interp, t)
    assert s.at(-1.0).pos == tr.points[0]
    assert s.at(99.0).pos == tr.points[-1]
    for t in (-1.0, 0.0, tr.duration, 99.0):
        if t <= 0.0 or t >= tr.duration:
            assert s.at(t).vel == (0.0, 0.0, 0.0)
            assert s.at(t).acc == (0.0, 0.0, 0.0)


@pytest.mark.parametrize('interp', ['linear', 'pchip'])
def test_position_is_continuous(interp):
    tr = parse_csv(ZIGZAG)
    s = Sampler(tr, interp)
    eps = 1e-7
    for t in tr.times[1:-1]:
        a, b = s.at(t - eps).pos, s.at(t + eps).pos
        assert close(a, b, 1e-5), (interp, t, a, b)


def test_pchip_velocity_is_continuous_and_zero_at_ends():
    tr = parse_csv(ZIGZAG)
    s = Sampler(tr, 'pchip')
    eps = 1e-7
    for t in tr.times[1:-1]:
        assert close(s.at(t - eps).vel, s.at(t + eps).vel, 1e-4), t
    assert close(s.at(1e-9).vel, (0, 0, 0), 1e-6)
    assert close(s.at(tr.duration - 1e-9).vel, (0, 0, 0), 1e-6)


@pytest.mark.parametrize('interp', ['linear', 'pchip'])
def test_velocity_is_the_derivative_of_position(interp):
    tr = parse_csv(ZIGZAG)
    s = Sampler(tr, interp)
    h = 1e-6
    for k in range(1, 90):
        t = tr.duration * k / 90.0
        if any(abs(t - tk) < 1e-3 for tk in tr.times):
            continue
        fd = tuple((s.at(t + h).pos[a] - s.at(t - h).pos[a]) / (2 * h) for a in range(3))
        assert close(fd, s.at(t).vel, 1e-5), (interp, t)
        if interp == 'pchip':
            fa = tuple((s.at(t + h).vel[a] - s.at(t - h).vel[a]) / (2 * h) for a in range(3))
            assert close(fa, s.at(t).acc, 1e-3), t


def test_linear_carries_no_acceleration_feed_forward():
    s = Sampler(parse_csv(SQUARE), 'linear')
    assert all(s.at(t / 10).acc == (0.0, 0.0, 0.0) for t in range(0, 170))


def test_pchip_never_leaves_the_waypoint_range_per_axis():
    tr = parse_csv(ZIGZAG)
    s = Sampler(tr, 'pchip')
    for i in range(len(tr) - 1):
        t0, t1 = tr.times[i], tr.times[i + 1]
        for k in range(201):
            p = s.at(t0 + (t1 - t0) * k / 200).pos
            for a in range(3):
                lo = min(tr.points[i][a], tr.points[i + 1][a])
                hi = max(tr.points[i][a], tr.points[i + 1][a])
                assert lo - 1e-12 <= p[a] <= hi + 1e-12, (i, k, a, p)


def test_pchip_on_the_geofence_edge_stays_inside():
    # Waypoints ON the box face; a Catmull-Rom spline would bulge past x = 1.75.
    tr = parse_csv('x,y,z,t\n0,0,0.5,0\n1.75,0,0.5,8\n1.75,1,0.5,12\n1.0,1.4,0.5,16\n'
                   '0,0,0.5,28\n')
    check_sampled_limits(Sampler(tr, 'pchip'), Limits())
    xs = [Sampler(tr, 'pchip').at(k * 0.01).pos[0] for k in range(2801)]
    assert max(xs) == 1.75


def test_offset_translates_everything():
    tr = parse_csv(SQUARE)
    s0 = Sampler(tr, 'linear')
    s1 = Sampler(tr, 'linear', offset=(0.2, -0.1, 0.05))
    for t in (0.0, 1.3, 7.9, 16.0):
        a, b = s0.at(t), s1.at(t)
        assert close(b.pos, (a.pos[0] + 0.2, a.pos[1] - 0.1, a.pos[2] + 0.05), 1e-12)
        assert b.vel == a.vel


def test_sample_times_rate_math():
    ts = sample_times(16.0, 50.0)
    assert len(ts) == 801
    assert ts[0] == 0.0 and ts[-1] == 16.0
    assert all(abs((b - a) - 0.02) < 1e-12 for a, b in zip(ts, ts[1:]))
    ts = sample_times(1.03, 10.0)             # not a multiple: exact end appended
    assert ts[-1] == 1.03 and len(ts) == 12
    assert ts[-2] == pytest.approx(1.0)
    ts = sample_times(0.3, 10.0)               # float fuzz: 0.3 * 10 = 2.9999...
    assert len(ts) == 4 and ts[-1] == 0.3
    with pytest.raises(TrajError):
        sample_times(1.0, 0.0)
    with pytest.raises(TrajError):
        sample_times(1.0, float('nan'))


def test_setpoint_table_is_deterministic_and_ends_on_the_last_waypoint():
    tr = parse_csv(SQUARE)
    a = setpoint_table(Sampler(tr, 'linear'), 50.0)
    b = setpoint_table(Sampler(tr, 'linear'), 50.0)
    assert a == b
    assert a[0].pos == (0.0, 0.0, 0.5)
    assert a[-1].pos == (0.0, 0.0, 0.5) and a[-1].t == 16.0
    assert a[100].pos == pytest.approx((0.5, 0.0, 0.5))   # t = 2 s, mid first leg
    assert a[100].vel == pytest.approx((0.25, 0.0, 0.0))


def test_pchip_peak_speed_refused_even_when_the_polyline_passes():
    # Secants 2.25 m/s, but the smooth curve peaks above v_max 2.5.
    tr = parse_csv('x,y,z,t\n0,0,0.5,0\n0.9,0,0.5,0.4\n1.35,0,0.5,0.6\n1.35,0,0.5,0.8\n')
    lim = Limits(a_max=250.0)
    check_sampled_limits(Sampler(tr, 'linear'), lim)          # polyline: fine
    with pytest.raises(TrajError) as ei:
        check_sampled_limits(Sampler(tr, 'pchip'), lim)
    assert ei.value.code == 'speed'


def test_pchip_accel_refused():
    tr = parse_csv('x,y,z,t\n0,0,0.5,0\n0.4,0,0.5,1\n0.4,0,0.5,2\n')
    with pytest.raises(TrajError) as ei:
        check_sampled_limits(Sampler(tr, 'pchip'), Limits(v_max=5.0, a_max=1.0))
    assert ei.value.code == 'accel'


def test_bad_interp():
    with pytest.raises(TrajError) as ei:
        Sampler(parse_csv(SQUARE), 'spline')
    assert ei.value.code == 'bad_interp'


def test_path_length():
    assert path_length(Sampler(parse_csv(SQUARE), 'linear')) == pytest.approx(4.0)
    # pchip hits every waypoint, so it is at least the polyline length.
    assert path_length(Sampler(parse_csv(SQUARE), 'pchip')) >= 4.0 - 1e-9
    circle = ['x,y,z,t'] + [f'{math.cos(2 * math.pi * k / 72)},{math.sin(2 * math.pi * k / 72)},'
                            f'0.5,{k * 0.5}' for k in range(73)]
    assert path_length(Sampler(parse_csv('\n'.join(circle)), 'linear')) == \
        pytest.approx(2 * math.pi, rel=1e-3)
