"""Setup script for isaacgym"""

import sys
import os

from setuptools import setup, find_packages

def collect_files(target_dir):
    file_list = []
    for (root, dirs, files) in os.walk(target_dir,followlinks=True):
        for filename in files:
            file_list.append(os.path.join('..', root, filename))
    return file_list

def _do_setup():
    root_dir = os.path.dirname(os.path.realpath(__file__))

    packages = find_packages(".")
    print(packages)

    #
    # TODO: do something more clever to collect only the bindings for the active versions of Python
    #

    package_files = []
    if sys.platform.startswith("win"):
        package_files = package_files + collect_files("isaacgym/_bindings/windows-x86_64")
    elif sys.platform.startswith("linux"):
        package_files = package_files + collect_files("isaacgym/_bindings/linux-x86_64")

    setup(name='isaacgym',
          version='1.0.preview3',
          description='GPU-accelerated simulation and reinforcement learning toolkit',
          author='NVIDIA CORPORATION',
          author_email='',
          url='http://developer.nvidia.com/isaac-gym',
          license='Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.',
          packages=packages,
          package_data={
              "isaacgym": package_files
          },
          python_requires='>=3.6,<3.9',
          install_requires = [
              "torch>=1.8.0",
              "torchvision>=0.9.0",
              "numpy>=1.16.4",
              "scipy>=1.5.0",
              "pyyaml>=5.3.1",
              "pillow",
              "imageio",
              "ninja",
          ],
         )

_do_setup()
