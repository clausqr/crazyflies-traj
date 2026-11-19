"""
The fleet config format (fleet.py): the sample loads as one player per vehicle,
YAML ints for double parameters become floats, and a malformed entry is refused.
"""

import os

import pytest
import yaml

from crazyflies_traj.fleet import vehicles
from crazyflies_traj.trajectory import TrajError

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_sample_fleet_is_the_three_drone_circle():
    """The vehicles and their files; other per-vehicle parameters (speedup, ...) are the
    operator's live tuning and not part of the contract."""
    with open(os.path.join(ROOT, 'trajectories', 'fleet.yaml')) as f:
        fleet = vehicles(yaml.safe_load(f))
    assert [(ns, p['csv_path']) for ns, p in fleet] == [
        ('cf1', 'trajectories/fleet/circle_1m_0.csv'), ('cf2', 'trajectories/fleet/circle_1m_120.csv'),
        ('cf3', 'trajectories/fleet/circle_1m_240.csv')]


def test_ints_for_double_parameters_become_floats():
    (_, p), = vehicles({'/cf1/traj_player': {'ros__parameters': {
        'csv_path': '/traj/a.csv', 'rate_hz': 50, 'acc_ff': True}}})
    assert p['rate_hz'] == 50.0 and isinstance(p['rate_hz'], float)
    assert p['acc_ff'] is True


@pytest.mark.parametrize('doc', [
    {'cf1': {'ros__parameters': {'csv_path': '/traj/a.csv'}}},
    {'/cf1/other_node': {'ros__parameters': {'csv_path': '/traj/a.csv'}}},
    {'/cf1/traj_player': {'csv_path': '/traj/a.csv'}},
    {'/cf1/traj_player': {'ros__parameters': {'frame': 'relative'}}},
])
def test_malformed_entry_is_refused(doc):
    with pytest.raises(TrajError) as e:
        vehicles(doc)
    assert e.value.code == 'bad_fleet'
