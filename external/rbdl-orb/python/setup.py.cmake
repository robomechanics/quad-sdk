#!/usr/bin/env python

from distutils.core import setup
from distutils.extension import Extension
from distutils.sysconfig import get_python_lib
from Cython.Distutils import build_ext
from Cython.Build import cythonize

import os
import sys
import numpy as np

if not os.path.exists('rbdl.so'):
	print("""The setup.py script should be executed from the build directory.""")
	sys.exit(1)

lib_path = get_python_lib()[5:]

libs = ["rbdl.so"]

muscleaddon = "@RBDL_BUILD_ADDON_MUSCLE@"

if muscleaddon == "ON":
	libs.append("rbdlmuscle.so")

setup(
	name='rbdl',
	author='Felix Richter',
	author_email='orb@felixrichter.tech',
	description='Python wrapper for RBDL - the Rigid Body Dynamics Library',
	license='zlib',
	version='${RBDL_VERSION_MAJOR}.${RBDL_VERSION_MINOR}.${RBDL_VERSION_PATCH}',
    url='https://rbdl.github.io/',
	data_files = [(lib_path, libs)],
)
