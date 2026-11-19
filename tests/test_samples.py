"""The shipped CSVs: every good sample passes the default limits, every bad one
is refused with the code its file name carries (bad/<code>[__variant].csv)."""

import glob
import os

import pytest

from crazyflies_traj.plan import check_offline
from crazyflies_traj.trajectory import Limits, load_csv, TrajError

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TRAJ = os.path.join(ROOT, 'trajectories')
GOOD = sorted(glob.glob(os.path.join(TRAJ, '*.csv')))
BAD = sorted(glob.glob(os.path.join(TRAJ, 'bad', '*.csv')))

# Flown on hardware (linear, relative frame). Dense 20 ms rows after 4 s lead-in
# rows: fine as a polyline, but pchip turns the coarse-to-dense junction into an
# acceleration spike far above a_max, so they are linear-only samples. They end
# on the floor margin (0, 0, z_floor) instead of the hover point.
LISSAJOUS = {'spherical_lissajous_slow.csv', 'spherical_lissajous_fast.csv'}
GOOD_INTERP = [pytest.param(p, i, id=f'{i}-{os.path.basename(p)}')
               for i in ('linear', 'pchip') for p in GOOD
               if not (i == 'pchip' and os.path.basename(p) in LISSAJOUS)]


def test_the_samples_exist():
    names = {os.path.basename(p) for p in GOOD}
    assert {'square_1m.csv', 'circle_r05.csv', 'figure8.csv'} | LISSAJOUS <= names
    assert len(BAD) >= 12


@pytest.mark.parametrize('path,interp', GOOD_INTERP)
def test_good_samples_pass_the_defaults(path, interp):
    check_offline(load_csv(path), Limits(), 'absolute', interp)


@pytest.mark.parametrize('name', sorted(LISSAJOUS))
def test_lissajous_samples_are_refused_under_pchip(name):
    with pytest.raises(TrajError) as ei:
        check_offline(load_csv(os.path.join(TRAJ, name)), Limits(), 'absolute', 'pchip')
    assert ei.value.code == 'accel', str(ei.value)


@pytest.mark.parametrize('path', GOOD, ids=os.path.basename)
def test_good_samples_start_and_end_at_rest_over_the_origin(path):
    tr = load_csv(path)
    end_z = Limits().z_floor if os.path.basename(path) in LISSAJOUS else 0.5
    assert tr.points[0] == pytest.approx((0.0, 0.0, 0.5))
    assert tr.points[-1] == pytest.approx((0.0, 0.0, end_z), abs=1e-4)


@pytest.mark.parametrize('path', BAD, ids=os.path.basename)
def test_bad_samples_refused_with_their_code(path):
    code = os.path.basename(path)[:-4].split('__')[0]
    with pytest.raises(TrajError) as ei:
        check_offline(load_csv(path), Limits(), 'absolute', 'linear')
    assert ei.value.code == code, str(ei.value)


def test_generator_is_reproducible(tmp_path):
    import shutil
    import subprocess
    import sys
    for name in ('generate.py',):
        shutil.copy(os.path.join(TRAJ, name), tmp_path / name)
    subprocess.run([sys.executable, str(tmp_path / 'generate.py')], check=True)
    for name in ('square_1m.csv', 'circle_r05.csv', 'figure8.csv'):
        with open(os.path.join(TRAJ, name)) as a, open(tmp_path / name) as b:
            assert a.read() == b.read(), f'{name} drifted from generate.py; rerun it'


def test_fast_lissajous_is_the_slow_one_at_twice_the_speed(tmp_path):
    import shutil
    import subprocess
    import sys
    (tmp_path / 'trajectories').mkdir()
    shutil.copy(os.path.join(TRAJ, 'spherical_lissajous_slow.csv'), tmp_path / 'trajectories')
    subprocess.run([sys.executable, os.path.join(ROOT, 'tools', 'modify_speed.py')],
                   cwd=tmp_path, check=True)
    name = 'spherical_lissajous_fast.csv'
    with open(os.path.join(TRAJ, name)) as a, open(tmp_path / 'trajectories' / name) as b:
        assert a.read() == b.read(), f'{name} drifted from tools/modify_speed.py; rerun it'
    slow = load_csv(os.path.join(TRAJ, 'spherical_lissajous_slow.csv'))
    fast = load_csv(os.path.join(TRAJ, name))
    assert fast.points == slow.points
    assert fast.duration == pytest.approx(slow.duration / 2)
