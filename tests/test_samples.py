"""The shipped CSVs and fleet yamls all load."""

import glob
import os
import shutil
import subprocess
import sys

import pytest

from crazyflies_traj.trajectory import load_csv

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TRAJ = os.path.join(ROOT, 'trajectories')
CSVS = sorted(glob.glob(os.path.join(TRAJ, '*.csv')) + glob.glob(os.path.join(TRAJ, 'fleet', '*.csv'))
              + glob.glob(os.path.join(TRAJ, 'examples', '*.csv')))
FLEETS = [os.path.join(TRAJ, 'fleet.yaml'), os.path.join(TRAJ, 'examples', 'circulo-3-drones_fleet.yaml')]


@pytest.mark.parametrize('path', CSVS, ids=os.path.basename)
def test_sample_loads(path):
    load_csv(path)


@pytest.mark.parametrize('path', FLEETS, ids=os.path.basename)
def test_fleet_yaml(path):
    yaml = pytest.importorskip('yaml')
    with open(path) as f:
        fleet = yaml.safe_load(f)
    for key, node in fleet.items():
        assert key.startswith('/') and key.endswith('/traj_player') and key.count('/') == 2
        csv_path = os.path.join(ROOT, node['ros__parameters']['csv_path'])
        assert os.path.exists(csv_path)
        load_csv(csv_path)


def test_generator_is_reproducible(tmp_path):
    shutil.copy(os.path.join(TRAJ, 'generate.py'), tmp_path / 'generate.py')
    subprocess.run([sys.executable, str(tmp_path / 'generate.py')], check=True)
    for name in ('square_1m.csv', 'circle_r05.csv', 'figure8.csv'):
        with open(os.path.join(TRAJ, name)) as a, open(tmp_path / name) as b:
            assert a.read() == b.read(), f'{name} drifted from generate.py; rerun it'


def test_fast_lissajous_is_the_slow_one_at_twice_the_speed(tmp_path):
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
