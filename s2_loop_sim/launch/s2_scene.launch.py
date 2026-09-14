"""Start Gazebo, the ROS/Gazebo bridge, the movement engine, and a random scene.

`ros2 launch` imports this module, calls generate_launch_description(), and runs
the actions it gets back; nothing here starts a process itself. So the module
level code runs once, before anything exists, which is why the layout can be
randomised inline. Where things go is decided by s2_loop_sim.layout; this file
only turns that answer into actions.

Paths come from the installed share/ tree rather than the source checkout.
GZ_SIM_RESOURCE_PATH is what Gazebo searches to resolve the model:// URIs in the
world file, so it is set before Gazebo starts. The bridge argument uses Gazebo's
`endpoint@ros_type` form. Models cannot be created until Gazebo is up and there
is no readiness signal to wait on, so SPAWN_DELAY is a guess; raise it if models
ever go missing at startup. MOVEMENT_START_DELAY leaves the spawned models a
moment to settle before the vehicle starts driving, and follows SPAWN_DELAY up.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node

from s2_loop_sim.constants import (
    MOVEMENT_START_DELAY,
    SET_POSE_SERVICE,
    SPAWN_DELAY,
    WORLD_NAME,
)
from s2_loop_sim.layout import random_layout, waypoint_coordinates


def create_model(models_path, placement):
    """`ros_gz_sim create` adds one model to the running world, then exits."""
    model, index, spot, z = placement

    return Node(
        package='ros_gz_sim', executable='create',
        arguments=[
            '-world', WORLD_NAME,
            '-file', os.path.join(models_path, model, f'{model}.sdf'),
            '-name', f'{model}_{index}',
            '-x', str(spot.x), '-y', str(spot.y), '-z', str(z),
        ],
    )


# `ros2 launch` imports this module and calls this function by name.
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

        TimerAction(period=SPAWN_DELAY,
                    actions=[create_model(models_path, placement)
                             for placement in placements]),

        TimerAction(period=MOVEMENT_START_DELAY,
                    actions=[Node(package='s2_loop_sim',
                                  executable='movement_engine.py',
                                  parameters=[{'waypoints':
                                               waypoint_coordinates(placements)}])]),

    ])
