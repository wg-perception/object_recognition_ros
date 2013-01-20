#!/usr/bin/env python
"""
This file launches an ORK actionlib server according to a config file
"""
from ecto.opts import scheduler_options
from object_recognition_core.utils.training_detection_args import create_parser, read_arguments
from object_recognition_ros.server import RecognitionServer
import rospy
import sys

if __name__ == '__main__':
    # create an ORK parser (it is special as it can read from option files)
    parser = create_parser()
    parser.description = ' This file executes an actionlib server that executes the ORK plasm contained in the ' \
                        'configuration file'

    # add ecto options
    scheduler_options(parser)

    ork_params, _args = read_arguments(parser)
    rospy.init_node('recognize_objects_server')
    server = RecognitionServer(ork_params)
    rospy.spin()
