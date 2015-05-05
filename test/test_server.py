#!/usr/bin/env python
"""
This file is meant to be used with test_client.py: it starts a recognition server for 10 seconds
"""
from object_recognition_core.utils.training_detection_args import read_arguments_from_string
from object_recognition_ros.server import RecognitionServer
import rospy
import sys
import ecto_ros
import roslib
import rospy

if __name__ == '__main__':
    ecto_ros.init([], "test_server_ecto_ros", False)

    rospy.init_node('test_server')
    ork_params = read_arguments_from_string("""
    sink:
      type: 'Publisher'
      module: 'object_recognition_ros.io'
    pipeline:
      type: ConstantDetector
      module: object_recognition_core.ecto_cells.pipelines
      outputs: [sink]
    """)

    server = RecognitionServer(ork_params)

    # timeout for the test
    rospy.sleep(10.0)
