"""
fleet: one traj_player per vehicle, from a ROS 2 params file (/<ns>/traj_player entries).

FLIES BY DEFAULT (by design: the fleet launch is the real
thing; the single-vehicle traj_player stays a dry run by default):

    ros2 launch crazyflies_traj fleet.launch.py config:=/traj/fleet.yaml              # flies
    ros2 launch crazyflies_traj fleet.launch.py config:=/traj/fleet.yaml fly:=false   # dry run

Each entry becomes node /<ns>/traj_player with its own parameters (csv_path and
any traj_player parameter as a per-vehicle override). The namespace keeps two
players from colliding as two bare /traj_player nodes; the node uses absolute
topic names and its foreign-publisher check compares (name, namespace), so
players on different vehicles never see each other. Vehicles are independent:
own gates, own /<ns>/traj_abort, no fleet abort. They DO start together: each
player holds at its first setpoint (streaming) and latches /<ns>/traj_ready
until every roster vehicle present on the graph is ready too (absent ones are
skipped; sync_timeout 30 s), then all stream their CSV on the same tick.

fly is a command-line decision: it is set here, after the file's values, so the
file can neither enable nor disable flight. ros2 launch does not propagate the players' exit
codes; read each node's DONE / ABORT lines.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from crazyflies_traj.fleet import vehicles
from crazyflies_traj.trajectory import TrajError

# (name, default, description). fly defaults to TRUE here (the fleet launch is
# the real thing); the repo guard on dry-run defaults covers traj_player.launch.py.
ARGS = [
    ('config', '', 'fleet params file: one /<ns>/traj_player entry per vehicle, '
                   'e.g. /traj/fleet.yaml'),
    ('fly', 'true', 'true = stream once each vehicle is airborne (the default); '
                    'false = DRY RUN for every vehicle (publishes nothing)'),
]


def launch_setup(context, *args, **kwargs):
    config = LaunchConfiguration('config').perform(context).strip()
    fly = LaunchConfiguration('fly').perform(context).strip().lower()
    if fly not in ('true', 'false'):
        raise RuntimeError(f"fly must be 'true' or 'false', got {fly!r}")
    if not config:
        raise RuntimeError('config:= is required (a fleet params file, e.g. /traj/fleet.yaml)')
    with open(config, encoding='utf-8') as f:
        doc = yaml.safe_load(f)
    try:
        fleet = vehicles(doc)
    except TrajError as exc:
        raise RuntimeError(f'{config}: {exc}') from exc
    if not fleet:
        raise RuntimeError(f'{config}: no /<ns>/traj_player entries')
    roster = ','.join(ns for ns, _ in fleet)     # the sync barrier's peer list
    # Each node gets a dict, not the file: FLOAT_KEYS are cast (an int for a
    # double parameter kills rclpy at declare time), and ns / fly come last.
    return [Node(
        package='crazyflies_traj',
        executable='traj_player',
        name='traj_player',
        namespace=ns,
        output='screen',
        emulate_tty=True,
        parameters=[{**params, 'ns': ns, 'fly': fly == 'true', 'fleet': roster}],
    ) for ns, params in fleet]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=t) for n, d, t in ARGS]
        + [OpaqueFunction(function=launch_setup)])
