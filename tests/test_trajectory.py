"""CSV loader + validator: the accepted shape and one seeded RED per refusal."""

import math

import pytest

from crazyflies_traj.trajectory import (check_limits, check_start, corner_accels, Limits,
                                        load_csv, parse_csv, TrajError)

GOOD = 'x,y,z,t\n0,0,0.5,0\n0.5,0,0.5,2\n0.5,0.5,0.5,4\n'


def refused(code, text, **kw):
    with pytest.raises(TrajError) as ei:
        parse_csv(text, **kw)
    assert ei.value.code == code, str(ei.value)
    return ei.value


def refused_limits(code, text, limits=Limits(), frame='absolute'):
    tr = parse_csv(text)
    with pytest.raises(TrajError) as ei:
        check_limits(tr, limits, frame)
    assert ei.value.code == code, str(ei.value)
    return ei.value


# ---- accepted shapes -----------------------------------------------------

def test_good_parses_and_passes_limits():
    tr = parse_csv(GOOD)
    assert tr.times == (0.0, 2.0, 4.0)
    assert tr.points[1] == (0.5, 0.0, 0.5)
    assert tr.duration == 4.0
    check_limits(tr, Limits())


def test_header_any_order_case_blanks_and_comments():
    text = '# a comment\n\n  T , Z, y ,X\n0,0.5,0,0\n  # indented comment\n2,0.5,0,0.5\n'
    tr = parse_csv(text)
    assert tr.points == ((0.0, 0.0, 0.5), (0.5, 0.0, 0.5))
    assert tr.times == (0.0, 2.0)


def test_headerless_needs_the_flag():
    text = '0,0,0.5,0\n0.5,0,0.5,2\n'
    e = refused('bad_header', text)
    assert '--no-header' in e.msg
    tr = parse_csv(text, header=False)
    assert tr.points[1] == (0.5, 0.0, 0.5)


def test_normalize_t_shifts_and_records_the_shift():
    text = 'x,y,z,t\n0,0,0.5,5\n0.5,0,0.5,7\n'
    refused('t_not_zero', text)
    tr = parse_csv(text, normalize_t=True)
    assert tr.times == (0.0, 2.0)
    assert tr.t_shift == 5.0


def test_pause_rows_are_allowed():
    # Same position, later t: a hover pause, not a duplicate.
    tr = parse_csv('x,y,z,t\n0,0,0.5,0\n0,0,0.5,1\n0.2,0,0.5,2\n')
    check_limits(tr, Limits())


# ---- structural refusals (seeded RED each) -------------------------------

@pytest.mark.parametrize('code,text', [
    ('too_few_rows', 'x,y,z,t\n0,0,0.5,0\n'),
    ('too_few_rows', 'x,y,z,t\n'),
    ('too_few_rows', ''),
    ('non_monotone_t', 'x,y,z,t\n0,0,0.5,0\n0.1,0,0.5,2\n0.2,0,0.5,1\n'),
    ('non_monotone_t', 'x,y,z,t\n0,0,0.5,0\n0.1,0,0.5,2\n0.2,0,0.5,2\n'),
    ('duplicate_row', 'x,y,z,t\n0,0,0.5,0\n0,0,0.5,0\n'),
    ('non_finite', 'x,y,z,t\n0,0,0.5,0\nnan,0,0.5,2\n'),
    ('non_finite', 'x,y,z,t\n0,0,0.5,0\n0,inf,0.5,2\n'),
    ('non_finite', 'x,y,z,t\n0,0,0.5,0\n0,0,-inf,2\n'),
    ('non_finite', 'x,y,z,t\n0,0,0.5,0\n0,0,0.5,NaN\n'),
    ('not_numeric', 'x,y,z,t\n0,0,0.5,0\n0,,0.5,2\n'),
    ('not_numeric', 'x,y,z,t\n0,0,0.5,0\n1e,0,0.5,2\n'),
    ('bad_row', 'x,y,z,t\n0,0,0.5,0\n0,0,0.5,2,9\n'),
    ('bad_header', 'x,y,z\n0,0,0.5\n'),
    ('bad_header', 'x,y,z,t,yaw\n0,0,0.5,0,0\n'),
    ('bad_header', 'x,x,z,t\n0,0,0.5,0\n1,0,0.5,2\n'),
    ('t_not_zero', 'x,y,z,t\n0,0,0.5,0.001\n0.1,0,0.5,2\n'),
    ('t_not_zero', 'x,y,z,t\n0,0,0.5,-1\n0.1,0,0.5,2\n'),
])
def test_structural_refusals(code, text):
    refused(code, text)


def test_refusal_names_the_file_line():
    e = refused('non_finite', 'x,y,z,t\n# c\n0,0,0.5,0\n0,nan,0.5,2\n', source='f.csv')
    assert 'f.csv:4' in e.msg


def test_unreadable_file(tmp_path):
    with pytest.raises(TrajError) as ei:
        load_csv(tmp_path / 'missing.csv')
    assert ei.value.code == 'unreadable'


# ---- limit refusals (seeded RED each) ------------------------------------

def test_geofence_x_y_and_ceiling():
    # Default box x, y in [-1.75, 1.75], z_max 2.0 above pad: just outside is
    # refused, the faces themselves pass.
    refused_limits('geofence', 'x,y,z,t\n0,0,0.5,0\n0,0.5,0.5,2\n0,1.8,0.5,6\n')
    refused_limits('geofence', 'x,y,z,t\n-1.2,0,0.5,0\n-1.8,0,0.5,2\n')
    refused_limits('geofence', 'x,y,z,t\n0,-1.2,0.5,0\n0,-1.8,0.5,2\n')
    refused_limits('geofence', 'x,y,z,t\n0,0,0.5,0\n0,0,1.0,2\n0,0,2.01,4\n')
    check_limits(parse_csv('x,y,z,t\n0,0,0.5,0\n0,0.5,0.5,2\n0,1.75,0.5,6\n'), Limits())
    check_limits(parse_csv('x,y,z,t\n-1.2,0,0.5,0\n-1.75,0,0.5,2\n'), Limits())
    check_limits(parse_csv('x,y,z,t\n0,-1.2,0.5,0\n0,-1.75,0.5,2\n'), Limits())
    check_limits(parse_csv('x,y,z,t\n0,0,0.5,0\n0,0,1.0,2\n0,0,2.0,4\n'), Limits())


def test_geofence_is_configurable():
    text = 'x,y,z,t\n0,0,0.5,0\n0.4,0,0.5,2\n'
    check_limits(parse_csv(text), Limits())
    refused_limits('geofence', text, Limits(x_max=0.3))


def test_below_floor_margin():
    refused_limits('below_floor', 'x,y,z,t\n0,0,0.5,0\n0,0,0.19,2\n')
    refused_limits('below_floor', 'x,y,z,t\n0,0,0.5,0\n0,0,0.4,2\n', Limits(z_floor=0.45))


def test_speed_limit():
    # v_max 2.5 m/s: 1.255 m in 0.5 s is 2.51 m/s, 1.25 m is exactly the limit.
    refused_limits('speed', 'x,y,z,t\n0,0,0.5,0\n1.255,0,0.5,0.5\n')
    check_limits(parse_csv('x,y,z,t\n0,0,0.5,0\n1.25,0,0.5,0.5\n'), Limits(a_max=20.0))
    refused_limits('speed', 'x,y,z,t\n0,0,0.5,0\n0.2,0,0.5,1\n', Limits(v_max=0.1))


def test_accel_limit_includes_start_from_hover():
    # 2 m/s straight from hover in a 0.5 s segment: 2 / 0.25 = 8 m/s^2 > a_max 4.
    refused_limits('accel', 'x,y,z,t\n0,0,0.5,0\n1.0,0,0.5,0.5\n1.0,0,0.5,3\n')
    # 1 m/s from hover reads exactly 4 m/s^2 and passes; 1.02 m/s reads 4.08.
    check_limits(parse_csv('x,y,z,t\n0,0,0.5,0\n0.5,0,0.5,0.5\n0.5,0,0.5,3\n'), Limits())
    refused_limits('accel', 'x,y,z,t\n0,0,0.5,0\n0.51,0,0.5,0.5\n0.51,0,0.5,3\n')


def test_accel_limit_at_a_corner():
    # 1.6 m/s then an instant reversal: 3.2 m/s over a 0.5 s window = 6.4 m/s^2.
    refused_limits('accel', 'x,y,z,t\n0,0,0.5,0\n0.4,0,0.5,1\n1.2,0,0.5,1.5\n'
                            '0.4,0,0.5,2\n0.4,0,0.5,4\n')


def test_too_long():
    refused_limits('too_long', 'x,y,z,t\n0,0,0.5,0\n0.1,0,0.5,121\n')


def test_bad_frame_and_bad_limits():
    refused_limits('bad_frame', GOOD, frame='body')
    refused_limits('bad_limits', GOOD, Limits(x_min=1.0, x_max=-1.0))
    refused_limits('bad_limits', GOOD, Limits(z_floor=0.0))
    refused_limits('bad_limits', GOOD, Limits(v_max=float('nan')))


def test_relative_extent_must_fit_the_box():
    # 3.6 m wide: fits no anchor in a 3.5 m box.
    text = 'x,y,z,t\n10,0,0.5,0\n11.8,0,0.5,4\n13.6,0,0.5,8\n'
    refused_limits('geofence', text, frame='relative')
    # Same shape, 3.4 m wide: fine relative, far outside absolute.
    text2 = 'x,y,z,t\n10,0,0.5,0\n11.7,0,0.5,4\n13.4,0,0.5,8\n'
    check_limits(parse_csv(text2), Limits(), 'relative')
    refused_limits('geofence', text2, frame='absolute')


# ---- start continuity -----------------------------------------------------

def test_start_continuity():
    assert check_start((0, 0, 0.55), (0.05, 0, 0.55), 0.15) == pytest.approx(0.05)
    with pytest.raises(TrajError) as ei:
        check_start((1.0, 1.0, 0.55), (0, 0, 0.55), 0.15)
    assert ei.value.code == 'start_far'
    with pytest.raises(TrajError):
        check_start((0, 0, 0.55), (0, 0, float('nan')), 0.15)


def test_corner_accels_circle_matches_centripetal():
    r, v = 0.5, 0.2
    n = 72
    period = 2 * math.pi * r / v
    rows = ['x,y,z,t']
    for k in range(n + 1):
        th = 2 * math.pi * k / n
        rows.append(f'{r * math.cos(th)},{r * math.sin(th)},0.5,{period * k / n}')
    a = corner_accels(parse_csv('\n'.join(rows)))
    assert a[n // 2] == pytest.approx(v * v / r, rel=0.01)
