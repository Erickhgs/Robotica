import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    models_path = os.path.join(pkg_share, 'models')
    model_file = os.path.join(models_path, 'my_robot', 'model.sdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        )
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot', '-file', model_file, '-x', '0', '-y', '0', '-z', '0'],
        output='screen'
    )

    return LaunchDescription([
        AppendEnvironmentVariable('GAZEBO_MODEL_PATH', '/usr/share/gazebo-11/models'),
        AppendEnvironmentVariable('GAZEBO_MODEL_PATH', models_path),
        gazebo,
        spawn_entity,
    ])
