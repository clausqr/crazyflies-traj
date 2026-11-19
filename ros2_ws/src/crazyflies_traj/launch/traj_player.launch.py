"""Stream a CSV trajectory: ros2 launch crazyflies_traj traj_player.launch.py csv_path:=/traj/square_1m.csv"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

STRINGS = [('csv_path', ''), ('ns', 'cf1'), ('yaw', 'fixed')]
FLOATS = [('rate_hz', '50.0'), ('speedup', '1.0'), ('hold_s', '3.0')]


def generate_launch_description():
    args = [DeclareLaunchArgument(n, default_value=d) for n, d in STRINGS + FLOATS]
    params = {n: LaunchConfiguration(n) for n, _ in STRINGS}
    params.update({n: ParameterValue(LaunchConfiguration(n), value_type=float)
                   for n, _ in FLOATS})
    node = Node(package='crazyflies_traj', executable='traj_player',
                name='traj_player', output='screen', parameters=[params])
    return LaunchDescription(args + [node])
