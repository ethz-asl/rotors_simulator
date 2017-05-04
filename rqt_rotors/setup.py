#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['rqt_rotors'],
    package_dir={'': 'src'},
    scripts=['scripts/hil_plugin']
)

setup(**setup_args)
