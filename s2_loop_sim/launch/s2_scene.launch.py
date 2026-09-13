import math
import os
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def sample_position(occupied, radius):
    while True:
        x, y = random.uniform(-4.5, 4.5), random.uniform(-4.5, 4.5)
        if all(math.hypot(x - ox, y - oy) > radius + other + 0.3
               for ox, oy, other in occupied):
            occupied.append((x, y, radius))
            return x, y


def generate_launch_description():
    pkg_dir = get_package_share_directory('s2_loop_sim')
    world_path = os.path.join(pkg_dir, 'worlds', 's2_scene.sdf')
    models_path = os.path.join(pkg_dir, 'models')
    occupied = [(0.0, 0.0, 0.5)]
    entities = []

    for kind, count, radius, height in (
        ('sphere_obstacle', 5, 0.45, 0.45),
        ('waypoint_star', 5, 0.5, 0.06),
    ):
        model = os.path.join(models_path, kind, f'{kind}.sdf')
        for index in range(1, count + 1):
            x, y = sample_position(occupied, radius)
            entities.append(Node(
                package='ros_gz_sim', executable='create',
                arguments=['-world', 's2_scene', '-file', model,
                           '-name', f'{kind}_{index}',
                           '-x', str(x), '-y', str(y), '-z', str(height)],
            ))

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_path),
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
            output='screen',
        ),
        TimerAction(period=2.0, actions=entities),
    ])
