#!/usr/bin/env python
from distutils.core import setup

setup(name='Object recognition modules for ROS',
      version='1.0.0',
      description='The ROS modules of object recognition',
      packages=['object_recognition_ros', 'object_recognition_ros.io',
                'object_recognition_ros.io', 'object_recognition_ros.io.source',
                'object_recognition_ros.io.sink'],
      package_dir={'':'python'}
)
