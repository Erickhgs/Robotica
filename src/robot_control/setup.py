from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'simple-pid>=2.0.0,<3.0.0'],
    zip_safe=True,
    maintainer='Erick',
    maintainer_email='hiagoe121@gmail.com',
    description='Controle de pose, missão e trajetória em 8',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'pose_controller = robot_control.pose_node:main',
        'mission_executor = robot_control.mission_node:main',
        'figure_eight = robot_control.trajectory_node:main',
        'kinematic_sim = robot_control.simulator_node:main',
        'analyze_results = robot_control.analyze:main',
    ]},
)
