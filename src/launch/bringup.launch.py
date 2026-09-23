import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'my_robot_pkg'

    pkg_share = get_package_share_directory(package_name)


    gazebo_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_share,
                'launch',
                'gazebo_urdf.launch.py'
            )
        )
    )


    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '60'
        ],
        output='screen'
    )


    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '60'
        ],
        output='screen'
    )

    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='xterm -e',
        output='screen'
    )


    return LaunchDescription([

        gazebo_robot,

        TimerAction(
            period=4.0,
            actions=[
                joint_state_broadcaster
            ]
        ),

        TimerAction(
            period=6.0,
            actions=[
                diff_drive_controller
            ]
        ),

        TimerAction(
            period=8.0,
            actions=[
                teleop
            ]
        )

    ])