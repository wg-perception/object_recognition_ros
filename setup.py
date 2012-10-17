#!/usr/bin/env python
from distutils.core import setup

setup(name='Object recognition core',
      version='1.0.0',
      description='The core of object recognition',
      packages=['object_recognition_ros.io.ros', 'object_recognition_ros.io.ros.source',
                'object_recognition_ros.io.ros.sink'],
      package_dir={'':'python'}
)
