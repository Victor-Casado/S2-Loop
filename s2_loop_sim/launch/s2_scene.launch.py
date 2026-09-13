"""Start Gazebo, the ROS/Gazebo bridge, the movement engine, and a random scene.

`ros2 launch` imports this module, calls generate_launch_description(), and runs
the actions it gets back; nothing here starts a process itself. So the module
level code runs once, before anything exists, which is why the layout can be
randomised inline.

Paths come from the installed share/ tree rather than the source checkout.
GZ_SIM_RESOURCE_PATH is what Gazebo searches to resolve the model:// URIs in the
world file, so it is set before Gazebo starts. The bridge argument uses Gazebo's
`endpoint@ros_type` form. Models cannot be created until Gazebo is up and there
is no readiness signal to wait on, so SPAWN_DELAY is a guess; raise it if models
ever go missing at startup.
"""
import math
import os
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node

from s2_loop_sim.constants import (
    ARENA_HALF_SIZE,
    MIN_GAP,
    PARKED_VEHICLE,
    SET_POSE_SERVICE,
    SPAWN_DELAY,
    SPAWNS,
    WORLD_NAME,
    Circle,
)


def overlaps(spot, other):
    """True when two footprints leave less than MIN_GAP of clear ground between."""
    return (math.hypot(spot.x - other.x, spot.y - other.y)
            <= spot.radius + other.radius + MIN_GAP)


def free_spot(taken, radius):
    """A random footprint in the arena that overlaps nothing in `taken`."""
    while True:
        spot = Circle(random.uniform(-ARENA_HALF_SIZE, ARENA_HALF_SIZE),
                      random.uniform(-ARENA_HALF_SIZE, ARENA_HALF_SIZE),
                      radius)

        if not any(overlaps(spot, other) for other in taken):
            return spot


def random_layout():
    """Return (spawn, index, spot) for every model to create, none overlapping."""
    taken = [PARKED_VEHICLE]
    placements = []

    for spawn in SPAWNS:
        for index in range(1, spawn.count + 1):
            spot = free_spot(taken, spawn.radius)
            taken.append(spot)
            placements.append((spawn, index, spot))

    return placements


def star_coordinates(placements):
    """Flatten the waypoint star positions into command-line arguments."""
    coordinates = []
    for spawn, _, spot in placements:
        if spawn.model == 'waypoint_star':
            coordinates += [str(spot.x), str(spot.y)]

    return coordinates


def create_model(models_path, spawn, index, spot):
    """`ros_gz_sim create` adds one model to the running world, then exits."""
    return Node(
        package='ros_gz_sim', executable='create',
        arguments=[
            '-world', WORLD_NAME,
            '-file', os.path.join(models_path, spawn.model, f'{spawn.model}.sdf'),
            '-name', f'{spawn.model}_{index}',
            '-x', str(spot.x), '-y', str(spot.y), '-z', str(spawn.z),
        ],
    )


def generate_launch_description():
    package_path = get_package_share_directory('s2_loop_sim')
    world_path = os.path.join(package_path, 'worlds', f'{WORLD_NAME}.sdf')
    models_path = os.path.join(package_path, 'models')
    placements = random_layout()

    return LaunchDescription([

        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_path),

        ExecuteProcess(cmd=['gz', 'sim', '-r', world_path], output='screen'),

        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=[f'{SET_POSE_SERVICE}@ros_gz_interfaces/srv/SetEntityPose']),

        Node(package='s2_loop_sim', executable='movement_engine.py'),

        TimerAction(period=SPAWN_DELAY,
                    actions=[create_model(models_path, *placement)
                             for placement in placements]),

        TimerAction(period=SPAWN_DELAY,
                    actions=[Node(package='s2_loop_sim',
                                  executable='waypoint_driver.py',
                                  arguments=star_coordinates(placements))]),

    ])
