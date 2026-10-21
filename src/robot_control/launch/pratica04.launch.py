from datetime import datetime
import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo,
    OpaqueFunction, RegisterEventHandler, TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build_nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    def boolean(name):
        value = arg(name).lower()
        if value not in ('true', 'false'):
            raise ValueError(f'{name}: use true ou false.')
        return value == 'true'
    noise, task = arg('noise'), arg('task')
    if noise not in ('zero', 'low', 'high') or task not in ('sensors', 'eight'):
        raise ValueError('noise=zero|low|high; task=sensors|eight')
    scale = {'zero': 0.0, 'low': 1.0, 'high': 8.0}[noise]
    omega, cycles, seed = float(arg('omega')), int(arg('cycles')), int(arg('seed'))
    if not math.isfinite(omega) or omega <= 0 or cycles < 1 or seed < 0:
        raise ValueError('omega > 0, cycles >= 1 e seed >= 0.')
    desc = Path(get_package_share_directory('robot_description'))
    ctrl = Path(get_package_share_directory('robot_control'))
    est = Path(get_package_share_directory('robot_state_estimation'))
    common = {'use_sim_time': True}
    nodes = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(desc/'launch/gazebo_urdf.launch.py')),
        launch_arguments={
            'sensors': 'true', 'noise_scale': str(scale),
            'controllers_file': str(desc/'config/controllers_pratica04.yaml'),
            'world': str(desc/'worlds/sensors.world'), 'gui': str(boolean('gui')).lower(),
            'spawn_yaw': str(math.atan2(4.0, 1.0) if task == 'eight' else 0.0),
            'extra_gazebo_args': f'--seed {seed}',
            'wheel_axis_y': '1',
        }.items()),
        Node(package='robot_state_estimation', executable='sensor_pose', name='sensor_pose',
             parameters=[str(est/'config/sensors.yaml'), common,
                         {'noise_scale': scale, 'random_seed': seed}], output='screen'),
    ]
    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager', '/controller_manager',
                               '--controller-manager-timeout', '90'], output='screen')
    joints, drive = spawner('joint_state_broadcaster'), spawner('diff_drive_controller')
    after_drive = []
    if task == 'eight':
        output = (Path(arg('output_dir')).expanduser()/
                  f'{noise}_seed{seed}_{datetime.now().strftime("%Y%m%d_%H%M%S_%f")}')
        output.mkdir(parents=True, exist_ok=False)
        (output/'run_metadata.json').write_text(json.dumps({
            'noise': noise, 'noise_scale': scale, 'seed': seed,
            'omega': omega, 'cycles': cycles, 'pose_source': '/localization/odom',
            'note': 'GPS+IMU direto, sem filtro; origem fixa do mundo sensors.world',
        }, indent=2), encoding='utf-8')
        nodes.extend([
            LogInfo(msg=f'Ensaio: {output}'),
            Node(package='robot_control', executable='ground_truth_recorder',
                 parameters=[common, {'output_dir': str(output)}], output='screen'),
        ])
        after_drive = [TimerAction(period=2.0, actions=[Node(
            package='robot_control', executable='figure_eight', name='figure_eight',
            parameters=[str(ctrl/'config/trajectory_sensors.yaml'), common,
                        {'trajectory.omega': omega, 'trajectory.cycles': cycles,
                         'output_dir': str(output)}],
            remappings=[('odom', '/localization/odom'), ('cmd_vel', '/cmd_vel')],
            output='screen')])]
    def after_exit(next_actions):
        def callback(event, _context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason='Falha ao ativar os controladores.'))]
            return next_actions
        return callback
    # Registrar eventos antes de iniciar spawners evita corrida de inicializacao.
    nodes.extend([
        RegisterEventHandler(OnProcessExit(target_action=joints, on_exit=after_exit([drive]))),
        RegisterEventHandler(OnProcessExit(target_action=drive, on_exit=after_exit(after_drive))),
        joints,
    ])
    if boolean('rviz'):
        nodes.append(Node(package='rviz2', executable='rviz2',
                          arguments=['-d', str(desc/'rviz/sensors.rviz')], parameters=[common]))
    return nodes


def generate_launch_description():
    args = [
        ('task', 'eight', 'eight: experimento; sensors: inspecao/teleoperacao'),
        ('noise', 'low', 'low ou high; zero somente para diagnostico'),
        ('seed', '42', 'Semente do Gazebo e do ruido de orientacao'),
        ('omega', '0.1', 'Frequencia angular da referencia, rad/s'),
        ('cycles', '1', 'Voltas completas; uma volta em 62.83 s com omega=0.1'),
        ('rviz', 'true', 'Abrir RViz configurado'), ('gui', 'true', 'Abrir interface do Gazebo'),
        ('output_dir', '~/pratica04_resultados', 'Pasta raiz dos ensaios'),
    ]
    return LaunchDescription([
        *(DeclareLaunchArgument(n, default_value=v, description=d) for n, v, d in args),
        OpaqueFunction(function=build_nodes),
    ])
