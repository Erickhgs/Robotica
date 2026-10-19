from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_state_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cauetnunes',
    maintainer_email='cauetnunesp@gmail.com',
    description='Localização e estimação de estado do robô',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
