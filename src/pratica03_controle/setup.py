from glob import glob
from setuptools import find_packages, setup

package_name = 'pratica03_controle'

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
    maintainer='Equipe da prática',
    maintainer_email='maintainer@example.com',
    description='Controle de pose e trajetória para ROS 2 Humble.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'pose_controller = pratica03_controle.pose_node:main',
        'mission_executor = pratica03_controle.mission_node:main',
        'figure_eight = pratica03_controle.trajectory_node:main',
        'kinematic_sim = pratica03_controle.simulator_node:main',
        'analyze_results = pratica03_controle.analyze:main',
    ]},
)
