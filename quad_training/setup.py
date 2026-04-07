from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'quad_training'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'time_sync_node = quad_training.time_sync_node:main',
            'episode_monitor_node = quad_training.episode_monitor_node:main',
            'wait_for_robot_node = quad_training.wait_for_robot_node:main',
        ],
    },
)
