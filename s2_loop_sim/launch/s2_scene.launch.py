"""Start the simulation: Gazebo, the ROS/Gazebo bridge, the movement engine, and
a freshly randomised set of obstacles and waypoints.

A launch file is ROS 2's answer to a shell script that starts several processes
at once. It does not start anything itself. `ros2 launch` imports this module,
calls generate_launch_description(), and executes the list of actions it gets
back. So the module-level Python here runs once, before any process exists,
which is why the random layout can just be computed inline.
"""
import math
import os
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def sample_position(occupied, radius):
    """Pick a free spot in the 9x9 m arena, rejecting anything that overlaps.

    `occupied` is a list of (x, y, radius) already taken, and it is appended to
    on success, so repeated calls keep the whole layout disjoint. The 0.3 m is
    breathing room on top of the two radii, so nothing spawns flush against a
    neighbour. This rejects and retries rather than solving for a layout, which
    is fine at ten objects and would not be at a thousand.
    """
    while True:
        x, y = random.uniform(-4.5, 4.5), random.uniform(-4.5, 4.5)
        if all(math.hypot(x - ox, y - oy) > radius + other + 0.3
               for ox, oy, other in occupied):
            occupied.append((x, y, radius))
            return x, y


def generate_launch_description():
    # colcon installs this package's launch, worlds and models directories into
    # a share/ tree somewhere outside the source checkout, and this call is how
    # you find that tree at runtime. Never read the files from the source dir.
    pkg_dir = get_package_share_directory('s2_loop_sim')
    world_path = os.path.join(pkg_dir, 'worlds', 's2_scene.sdf')
    models_path = os.path.join(pkg_dir, 'models')

    # The vehicle already sits at the origin, so reserve it with a generous
    # radius before anything else is placed.
    occupied = [(0.0, 0.0, 0.5)]
    entities = []

    # Five obstacles and five waypoints, positions drawn fresh every launch.
    # `height` is the z the model spawns at: half the sphere's diameter puts it
    # on the ground, and the star is flat so it barely clears it.
    for kind, count, radius, height in (
        ('sphere_obstacle', 5, 0.45, 0.45),
        ('waypoint_star', 5, 0.5, 0.06),
    ):
        model = os.path.join(models_path, kind, f'{kind}.sdf')
        for index in range(1, count + 1):
            x, y = sample_position(occupied, radius)
            # `create` is a one-shot command-line tool from ros_gz_sim: it
            # injects one model into a running world and exits. Names have to be
            # unique within the world, hence the index suffix.
            entities.append(Node(
                package='ros_gz_sim', executable='create',
                arguments=['-world', 's2_scene', '-file', model,
                           '-name', f'{kind}_{index}',
                           '-x', str(x), '-y', str(y), '-z', str(height)],
            ))

    # Actions run in the order listed, but each one only starts a process; they
    # do not wait for each other.
    return LaunchDescription([
        # Gazebo resolves the model:// URIs in the world file by searching this
        # path, so it has to be set before Gazebo starts.
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_path),
        # Gazebo itself. ExecuteProcess runs any binary, unlike Node, which is
        # for ROS nodes. -r starts the physics running instead of paused, and
        # output='screen' forwards its logs to this terminal.
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
            output='screen',
        ),
        # Gazebo and ROS 2 use different, incompatible transports. This bridge
        # process translates between them, and the argument says which endpoints
        # to expose: the string before @ is the name on both sides, the string
        # after is the ROS type to map it to. Without this line the movement
        # engine's set_pose client has nothing to call.
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            arguments=['/world/s2_scene/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
        ),
        # The movement engine. `executable` is the filename installed into
        # lib/s2_loop_sim by CMakeLists.txt, extension included.
        Node(package='s2_loop_sim', executable='movement_engine.py'),
        # Spawning has to wait for Gazebo to be up, and there is no readiness
        # signal to wait on, so this is a two second guess. If models are ever
        # missing at startup, this number is the first thing to raise.
        TimerAction(period=2.0, actions=entities),
    ])
