import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable


def generate_launch_description():
    pkg_dir = get_package_share_directory('s2_loop_sim')
    world_path = os.path.join(pkg_dir, 'worlds', 's2_scene.sdf')
    models_path = os.path.join(pkg_dir, 'models')

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_path),
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
            output='screen',
        ),
    ])