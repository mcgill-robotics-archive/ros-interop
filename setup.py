#!/usr/bin/env python

"""AUVSI SUAS Interoperability ROS client setup."""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

__author__ = "Anass Al"

d = generate_distutils_setup(
    packages=["interop"],
    package_dir={"": "src"}
)

setup(**d)
