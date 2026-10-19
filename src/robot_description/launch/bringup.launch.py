import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def spawner(controller):
    return Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            controller,
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        output='screen'
    )


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')

    gazebo_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo_urdf.launch.py')
        )
    )

    return LaunchDescription([
        gazebo_robot,
        TimerAction(period=4.0, actions=[spawner('joint_state_broadcaster')]),
        TimerAction(period=6.0, actions=[spawner('diff_drive_controller')]),
    ])
