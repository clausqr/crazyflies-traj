"""
traj_player: stream a CSV trajectory to the crazyflies-gs plant once airborne.

DRY RUN unless fly:=true. Dry run first, always:

    ros2 launch crazyflies_traj traj_player.launch.py csv_path:=/traj/square_1m.csv

prints the summary and every setpoint, then shadows the airborne gates against
the live graph and publishes nothing. Only after reviewing that output, with
preflight GO, the vehicle hovering (cfhw / HL takeoff) and the e-stop shell
ready, the operator adds fly:=true.

Heights in the CSV are ABOVE the pad z0 (latched /<ns>/pad_z0, or pin pad_z0).
Leave pad_z0 empty to take it from the plant.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# (name, default, description). fly defaults to false and must stay so: the
# repo guard tests/test_repo_guards.py fails if any default here enables flight.
ARGS = [
    ('csv_path', '', 'trajectory CSV (x, y, z, t; z above pad z0), e.g. /traj/square_1m.csv'),
    ('fly', 'false', 'false = DRY RUN (publishes nothing); true = stream once airborne'),
    ('ns', 'cf1', 'vehicle namespace (RR_CF_NAME / crazyflies.yaml robot name)'),
    ('rate_hz', '50.0', 'setpoint rate, 10..100 Hz'),
    ('frame', 'absolute', 'absolute (x, y = Motive world) | relative (anchored at trigger)'),
    ('interp', 'linear', 'linear | pchip (monotone cubic, zero end velocity)'),
    ('command', 'full_state', 'full_state (pos + vel feed-forward) | position (pos only)'),
    ('acc_ff', 'false', 'feed the pchip acceleration forward (full_state only)'),
    ('header', 'true', 'false: headerless CSV, columns x, y, z, t'),
    ('normalize_t', 'false', 'true: shift t so the first row is 0'),
    ('pad_z0', '', 'pin the pad height z0 (m, absolute); empty = latched /<ns>/pad_z0'),
    ('hold_s', '3.0', 'hold the last point this long, then stop streaming'),
    ('approach_v', '0.3', 'peak speed of the approach to the first setpoint, m/s'),
    ('shadow', 'true', 'dry run only: false = print and exit, do not watch the graph'),
]


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context).strip()

    def as_bool(name):
        v = arg(name).lower()
        if v not in ('true', 'false'):
            raise RuntimeError(f"{name} must be 'true' or 'false', got {v!r}")
        return v == 'true'

    params = {
        'csv_path': arg('csv_path'),
        'fly': as_bool('fly'),
        'ns': arg('ns'),
        'rate_hz': float(arg('rate_hz')),
        'frame': arg('frame'),
        'interp': arg('interp'),
        'command': arg('command'),
        'acc_ff': as_bool('acc_ff'),
        'header': as_bool('header'),
        'normalize_t': as_bool('normalize_t'),
        'hold_s': float(arg('hold_s')),
        'approach_v': float(arg('approach_v')),
        'shadow': as_bool('shadow'),
    }
    if arg('pad_z0'):
        params['pad_z0'] = float(arg('pad_z0'))
    return [Node(
        package='crazyflies_traj',
        executable='traj_player',
        name='traj_player',
        output='screen',
        emulate_tty=True,
        parameters=[params],
    )]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=t) for n, d, t in ARGS]
        + [OpaqueFunction(function=launch_setup)])
