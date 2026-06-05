from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'drone_system_pkg'
config_files = sorted(glob(os.path.join('..', '..', 'config', '*')))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', config_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yi',
    maintainer_email='ysun1@seattlu.edu',
    description='PX4 / MAVROS-oriented bird deterrence simulation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'coordinator_node = drone_system.coordinator_runtime:main',
            'centralized_coordinator_node = drone_system.centralized_coordinator_node:main',
            'birds_node = drone_system.birds_node:main',
            'chased_mask_aggregator_node = drone_system.chased_mask_aggregator_node:main',
            'visualization_node = drone_system.visualize_node:main',
            'perception_node = drone_system.perception_node:main',
            'obstacles_node = drone_system.obstacles_node:main',
        ],
    },
)