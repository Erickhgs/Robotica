import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Caminho absoluto para o seu arquivo SDF
    model_path = os.path.expanduser('~/Documents/robotica/src/model_description/model.sdf')

    # Inclui o launch padrão do Gazebo (inicia o mundo vazio)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # Nó para carregar (spawn) o modelo no Gazebo
    # spawn_entity = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=['-entity', 'my_robot', '-file', model_path, '-x', '0', '-y', '0', '-z', '0'],
    #     output='screen'
    # )

    return LaunchDescription([
        gazebo,
        #spawn_entity
    ])