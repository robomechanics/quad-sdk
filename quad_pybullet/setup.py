from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['robot_estimate'],
    package_dir={'': 'src'})

setup(**setup_args)
