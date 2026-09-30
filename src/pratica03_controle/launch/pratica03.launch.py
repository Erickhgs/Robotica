"""Seleciona uma única tarefa; remapeia os tópicos para o robô existente."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def as_bool(value):
    if value.lower() not in ('true', 'false'):
        raise ValueError('Use true ou false para argumentos booleanos.')
    return value.lower() == 'true'


def build_nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    task = arg('task')
    if task not in ('pose', 'mission', 'eight'):
        raise ValueError('task deve ser pose, mission ou eight.')
    demo = as_bool(arg('demo'))
    config = Path(arg('config_dir'))
    common = {
        'use_sim_time': False if demo else as_bool(arg('use_sim_time')),
        'frame_id': arg('frame_id'),
        'output_dir': arg('output_dir'),
    }
    # O demo usa tópicos próprios para não enviar movimento ao Gazebo por engano.
    odom = '/pratica03_demo/odom' if demo else arg('odom_topic')
    cmd = '/pratica03_demo/cmd_vel' if demo else arg('cmd_vel_topic')
    remaps = [('odom', odom), ('cmd_vel', cmd)]
    nodes = []
    if demo:
        nodes.append(Node(
            package='pratica03_controle', executable='kinematic_sim', name='kinematic_sim',
            parameters=[str(config/'simulator.yaml'), common], remappings=remaps, output='screen'))
    if task in ('pose', 'mission'):
        overrides = {'mission_mode': task == 'mission'}
        if arg('method'):
            overrides['method'] = arg('method')
        nodes.append(Node(
            package='pratica03_controle', executable='pose_controller', name='pose_controller',
            parameters=[str(config/'pose.yaml'), common, overrides],
            remappings=remaps, output='screen'))
        if task == 'mission':
            nodes.append(Node(
                package='pratica03_controle', executable='mission_executor', name='mission_executor',
                parameters=[str(config/'mission.yaml'), common],
                remappings=[('odom', odom)], output='screen'))
    else:
        overrides = {}
        for argument, parameter, convert in (
            ('mode', 'mode', str), ('kff', 'Kff', float),
            ('omega', 'trajectory.omega', float), ('cycles', 'trajectory.cycles', int),
        ):
            if arg(argument):
                overrides[parameter] = convert(arg(argument))
        nodes.append(Node(
            package='pratica03_controle', executable='figure_eight', name='figure_eight',
            parameters=[str(config/'trajectory.yaml'), common, overrides],
            remappings=remaps, output='screen'))
    return nodes


def generate_launch_description():
    config = str(Path(get_package_share_directory('pratica03_controle'))/'config')
    arguments = [
        ('task', 'mission', 'pose, mission ou eight'),
        ('demo', 'false', 'Simulação cinemática sem Gazebo, com relógio de parede'),
        ('use_sim_time', 'true', 'Usar /clock do Gazebo; ignorado no demo'),
        ('frame_id', 'odom', 'Frame comum à odometria e aos objetivos'),
        ('odom_topic', '/odom', 'Tópico nav_msgs/Odometry do robô'),
        ('cmd_vel_topic', '/cmd_vel', 'Tópico geometry_msgs/Twist'),
        ('output_dir', '~/pratica03_resultados', 'Pasta dos CSVs de cada execução'),
        ('config_dir', config, 'Diretório com os arquivos YAML'),
        ('method', '', 'Opcional: continuous ou three_maneuvers; vazio usa YAML'),
        ('mode', '', 'Opcional: open_loop ou feedback; vazio usa YAML'),
        ('kff', '', 'Opcional: ganho feedforward; vazio usa YAML'),
        ('omega', '', 'Opcional: frequência angular, rad/s; vazio usa YAML'),
        ('cycles', '', 'Opcional: número inteiro de voltas; vazio usa YAML'),
    ]
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in arguments),
        OpaqueFunction(function=build_nodes),
    ])
