# -*- coding: utf-8 -*-
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['dongle_bridge'],
    package_dir={'': 'src'},
    install_requires=[],
)

setup(**setup_args)
