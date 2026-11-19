"""One traj_player per vehicle: ros2 launch crazyflies_traj fleet.launch.py config:=fleet.yaml"""

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    with open(LaunchConfiguration('config').perform(context)) as f:
        cfg = yaml.safe_load(f)
    nodes = []
    for key, val in cfg.items():
        ns = key.strip('/').split('/')[0]
        params = {k: float(v) if isinstance(v, int) and not isinstance(v, bool) else v
                  for k, v in val['ros__parameters'].items()}
        nodes.append(Node(package='crazyflies_traj', executable='traj_player',
                          namespace=ns, name='traj_player', output='screen',
                          parameters=[{**params, 'ns': ns}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', description='fleet YAML: one /<ns>/traj_player block per vehicle'),
        OpaqueFunction(function=_nodes),
    ])
