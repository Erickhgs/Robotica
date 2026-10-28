from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    practice04 = Path(get_package_share_directory('robot_control'))/'launch/pratica04.launch.py'
    args = [
        ('task', 'eight', 'sensors: exercicio 1.1; eight: exercicio 2.1'),
        ('noise', 'low', 'low ou high; zero somente para diagnostico'),
        ('seed', '42', 'Semente do Gazebo e do ruido de orientacao'),
        ('omega', '0.1', 'Frequencia angular da referencia, rad/s'),
        ('cycles', '1', 'Voltas completas na trajetoria em 8'),
        ('rviz', 'true', 'Abrir RViz'), ('gui', 'true', 'Abrir interface do Gazebo'),
        ('output_dir', '~/pratica05_resultados', 'Pasta raiz dos ensaios'),
    ]
    return LaunchDescription([
        *(DeclareLaunchArgument(n, default_value=v, description=d) for n, v, d in args),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(practice04)),
            launch_arguments={
                **{n: LaunchConfiguration(n) for n, _, _ in args},
                'pose_source': 'filtered',
            }.items()),
    ])
