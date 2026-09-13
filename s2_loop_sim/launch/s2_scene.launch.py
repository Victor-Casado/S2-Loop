"""Start the simulation: Gazebo, the ROS/Gazebo bridge, the movement engine, and
a freshly randomised set of obstacles and waypoints.

A launch file is ROS 2's answer to a shell script that starts several processes
at once. It does not start anything itself. `ros2 launch` imports this module,
calls generate_launch_description(), and executes the list of actions it gets
back. So the module-level Python here runs once, before any process exists,
which is why the random layout can just be computed inline.

Every number this file uses lives in s2_loop_sim/constants.py.
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
    SPAWNS,
    SPAWN_DELAY,
    SET_POSE_SERVICE,
    VEHICLE_FOOTPRINT,
    WORLD_NAME,
)


def claim_position(taken, radius):
    """Pick a free spot in the arena, add it to `taken`, and return it.

    Mutating `taken` is the point: successive calls see everything placed so
    far, which is what keeps the layout disjoint. Each entry is (x, y, radius).

    This rejects and retries rather than solving for a layout. Fine for ten
    objects in an 81 square metre arena, and it would not be for a thousand.
    """
    while True:
        x = random.uniform(-ARENA_HALF_SIZE, ARENA_HALF_SIZE)
        y = random.uniform(-ARENA_HALF_SIZE, ARENA_HALF_SIZE)

        clear = all(
            math.hypot(x - other_x, y - other_y) > radius + other_radius + MIN_GAP
            for other_x, other_y, other_radius in taken)

        if clear:
            taken.append((x, y, radius))
            return x, y


def random_layout():
    """Yield (kind, index, x, y, z) for everything to spawn, nothing overlapping.

    The vehicle is already parked at the origin, so its footprint is reserved
    before anything else is placed.
    """
    taken = [VEHICLE_FOOTPRINT]

    for kind, count, radius, spawn_z in SPAWNS:
        for index in range(1, count + 1):
            x, y = claim_position(taken, radius)
            yield kind, index, x, y, spawn_z


def spawn_action(models_path, kind, index, x, y, z):
    """One model injected into the running world.

    `create` is a one-shot command-line tool from ros_gz_sim: it adds a single
    model to a world that is already up, then exits. Names have to be unique
    within the world, hence the index suffix.
    """
    return Node(
        package='ros_gz_sim', executable='create',
        arguments=[
            '-world', WORLD_NAME,
            '-file', os.path.join(models_path, kind, f'{kind}.sdf'),
            '-name', f'{kind}_{index}',
            '-x', str(x), '-y', str(y), '-z', str(z),
        ],
    )


def generate_launch_description():
    # colcon installs this package's launch, worlds and models directories into
    # a share/ tree outside the source checkout, and this call is how you find
    # that tree at runtime. Never read these files from the source directory.
    package_path = get_package_share_directory('s2_loop_sim')
    world_path = os.path.join(package_path, 'worlds', f'{WORLD_NAME}.sdf')
    models_path = os.path.join(package_path, 'models')

    spawns = [
        spawn_action(models_path, *placement) for placement in random_layout()
    ]

    # Actions start in the order listed, but each one only launches a process.
    # Nothing here waits for anything else.
    return LaunchDescription([

        # Gazebo resolves the model:// URIs in the world file by searching this
        # path, so it has to be set before Gazebo starts.
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_path),

        # Gazebo itself. ExecuteProcess runs any binary, unlike Node, which is
        # for ROS nodes. -r starts physics running rather than paused, and
        # output='screen' forwards Gazebo's logs to this terminal.
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
            output='screen',
        ),

        # Gazebo and ROS 2 use different, incompatible transports. This bridge
        # process translates between them, and its argument says which endpoints
        # to expose: the name before the @ is used on both sides, and the part
        # after it is the ROS type to map that endpoint to. Without this line the
        # movement engine's client has nothing to call.
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            arguments=[f'{SET_POSE_SERVICE}@ros_gz_interfaces/srv/SetEntityPose'],
        ),

        # The movement engine. `executable` is the filename installed into
        # lib/s2_loop_sim by CMakeLists.txt, extension included.
        Node(package='s2_loop_sim', executable='movement_engine.py'),

        # Spawning has to wait for Gazebo to be up, and there is no readiness
        # signal to wait on, so this is a timed guess.
        TimerAction(period=SPAWN_DELAY, actions=spawns),

    ])
