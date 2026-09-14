"""Start Gazebo on a freshly randomised world, the ROS/Gazebo bridge, and the engine.

`ros2 launch` imports this module, calls generate_launch_description(), and runs
the actions it gets back; nothing here starts a process itself. So the module
level code runs once, before anything exists, which is why the layout can be
randomised inline. Where things go is decided by s2_loop_sim.layout; this file
only turns that answer into a world file and a few actions.

Paths come from the installed share/ tree rather than the source checkout.
GZ_SIM_RESOURCE_PATH is what Gazebo searches to resolve the model:// URIs, so it
is set before Gazebo starts. The bridge argument uses Gazebo's
`endpoint@ros_type` form.
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node

from s2_loop_sim.constants import MOVEMENT_START_DELAY, SET_POSE_SERVICE, WORLD_NAME
from s2_loop_sim.layout import random_layout, waypoint_coordinates


def randomised_world(world_path, placements):
    """Write a copy of the world with every placed model already included in it.

    The models used to be added to the running world by `ros_gz_sim create`.
    That always reached the server, but the GUI builds its scene from a
    snapshot plus updates, and models arriving while it was still starting up
    could be missed: present in the simulation, absent from the viewport and
    the entity tree. Anything in the world file before Gazebo opens it cannot
    be missed that way.

    Returns the path to the copy, which Gazebo reads once at startup.
    """
    includes = '\n'.join(
        f'    <include>\n'
        f'      <uri>model://{model}</uri>\n'
        f'      <name>{model}_{index}</name>\n'
        f'      <pose>{spot.x} {spot.y} {z} 0 0 0</pose>\n'
        f'    </include>'
        for model, index, spot, z in placements)

    with open(world_path) as original:
        world = original.read()

    handle, randomised_path = tempfile.mkstemp(prefix=f'{WORLD_NAME}_', suffix='.sdf')
    with os.fdopen(handle, 'w') as copy:
        copy.write(world.replace('  </world>', f'{includes}\n  </world>'))

    return randomised_path


# `ros2 launch` imports this module and calls this function by name.
def generate_launch_description():
    package_path = get_package_share_directory('s2_loop_sim')
    world_path = os.path.join(package_path, 'worlds', f'{WORLD_NAME}.sdf')
    models_path = os.path.join(package_path, 'models')
    placements = random_layout()

    return LaunchDescription([

        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_path),

        ExecuteProcess(output='screen',
                       cmd=['gz', 'sim', '-r',
                            randomised_world(world_path, placements)]),

        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=[f'{SET_POSE_SERVICE}@ros_gz_interfaces/srv/SetEntityPose']),

        TimerAction(period=MOVEMENT_START_DELAY,
                    actions=[Node(package='s2_loop_sim',
                                  executable='movement_engine.py',
                                  parameters=[{'waypoints':
                                               waypoint_coordinates(placements)}])]),

    ])
