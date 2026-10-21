import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build_nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    pkg_share = get_package_share_directory('robot_description')
    gazebo_share = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file, mappings={
        'sensors': arg('sensors'), 'noise_scale': arg('noise_scale'),
        'controllers_file': arg('controllers_file'),
        'wheel_axis_y': arg('wheel_axis_y'),
    }).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'params_file': os.path.join(pkg_share, 'config', 'gazebo.yaml'),
            'world': arg('world'),
            'gui': arg('gui'),
            'extra_gazebo_args': arg('extra_gazebo_args'),
        }.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-topic', 'robot_description',
            '-z', '0.1',
            '-Y', arg('spawn_yaw'),
        ],
        output='screen'
    )

    return [
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ]


def generate_launch_description():
    share = get_package_share_directory('robot_description')
    args = [
        ('sensors', 'false'), ('noise_scale', '1.0'),
        ('controllers_file', os.path.join(share, 'config', 'controllers.yaml')),
        ('world', 'empty.world'), ('gui', 'true'), ('spawn_yaw', '0.0'),
        ('extra_gazebo_args', ''),
        ('wheel_axis_y', '-1'),
    ]
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in args),
        OpaqueFunction(function=build_nodes),
    ])
