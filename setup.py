#!/usr/bin/env python

from distutils.core import setup
from ecto.package import parse_package_for_distutils

d = parse_package_for_distutils()
d['packages'] = ['object_recognition_ros', 'object_recognition_ros.io',
                'object_recognition_ros.io', 'object_recognition_ros.io.source',
                'object_recognition_ros.io.sink']
d['package_dir'] = {'': 'python'}
d['install_requires'] = []

setup(**d)
