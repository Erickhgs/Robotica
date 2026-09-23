import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_name = 'my_robot_pkg'

    pkg_share = get_package_share_directory(package_name)

    gazebo_share = get_package_share_directory('gazebo_ros')

    urdf_file = os.path.join(
        pkg_share,
        'model_description',
        'my_robot',
        'my_robot.urdf'
    )

    with open(urdf_file, 'r') as file:
        robot_description = file.read()


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_share,
                'launch',
                'gazebo.launch.py'
            )
        )
    )


    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ]
    )


    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-topic', 'robot_description',

            '-z', '0.1'
        ],
        output='screen'
    )


    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot
    ])