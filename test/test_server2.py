#!/usr/bin/env python
"""
This file is meant to be used with test_server.py: it starts a recognition server for 5 seconds
"""
from object_recognition_core.utils.training_detection_args import read_arguments_from_string
from object_recognition_ros.server import RecognitionServer
import rospy
import sys
import ecto_ros

#if __name__ == '__main__':
#    rospy.init_node('test_server')
#    ork_params = read_arguments_from_string(open('config_detection_test.json'))
#    server = RecognitionServer(ork_params)
#    rospy.sleep(5.0)

import roslib
import rospy
from std_msgs.msg import String


if __name__ == '__main__':
    original_argv = sys.argv

    ecto_ros.init(original_argv, "server_test", False)

    rospy.init_node('test_server')
    ork_params = read_arguments_from_string(open('config_detection_test.json'))
    server = RecognitionServer(ork_params)
    while not rospy.is_shutdown():
        rospy.sleep(1.0)
