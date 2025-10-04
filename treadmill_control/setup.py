from setuptools import find_packages, setup

package_name = 'treadmill_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rml',
    maintainer_email='jbzh23@gmail.com',
    description='ROS2 wrapper for Bertec Treadmill Remote Control using Python',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'treadmill_control_node = treadmill_control.treadmill_control_node:main'
        ],
    },
)
