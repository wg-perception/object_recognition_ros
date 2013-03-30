# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of Willow Garage, Inc. nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""
Define an actionlib server for ORK
"""
from __future__ import print_function
from object_recognition_core.pipelines.plasm import create_plasm
from object_recognition_msgs.msg import RecognizedObjectArray
from object_recognition_msgs.msg._ObjectRecognitionAction import ObjectRecognitionAction
from object_recognition_msgs.msg._ObjectRecognitionResult import ObjectRecognitionResult
import actionlib
import rospy
import sys

DEFAULT_NODE_NAME = "object_recognition_server"

class RecognitionServer:
    """
    Main server that reads a config file, builds an actionlib server, reads an ecto plasm and run it when
    the actionlib server is queried
    """
    def __init__(self, ork_params):
        # create the plasm that will run the detection
        self.plasm = create_plasm(ork_params)
        self.plasm.configure_all()
        rospy.loginfo('ORK server configured')

        # the results or the object recognition pipeline
        self.recognition_result = None

        topics = ['recognized_object_array']

        for sink in ork_params.values():
            if 'recognized_object_array_topic' in sink:
                topics.append(sink['recognized_object_array_topic'])

        # subscribe to the output of the detection pipeline
        for topic in topics:
            rospy.Subscriber(topic, RecognizedObjectArray, self.callback_recognized_object_array)
            rospy.loginfo('Subscribed to the ' + topic + ' topic.')

        # look for a cell that contains a cropper to select the ROI
        self.cropper = None
        for cell in self.plasm.cells():
            if 'crop_enabled' in cell.params:
                self.cropper = cell

        # actionlib stuff
        self.server = actionlib.SimpleActionServer('recognize_objects', ObjectRecognitionAction, self.execute, False)
        self.server.start()
        rospy.loginfo('ORK server started')

    def callback_recognized_object_array(self, data):
        self.recognition_result = data
    
    def execute(self, goal):
        if self.cropper is not None:
            self.cropper.params.crop_enabled = goal.use_roi
            if goal.use_roi:
                if len(goal.filter_limits) == 6:
                    self.cropper.params.x_min = goal.filter_limits[0]
                    self.cropper.params.x_max = goal.filter_limits[1]
                    self.cropper.params.y_min = goal.filter_limits[2]
                    self.cropper.params.y_max = goal.filter_limits[3]
                    self.cropper.params.z_min = goal.filter_limits[4]
                    self.cropper.params.z_max = goal.filter_limits[5]
                else:
                    print('WARNING: goal.use_roi is enabled but filter_limits doesn\'t have size 6 [x_min, x_max, y_min, y_max, z_min, z_max]. Roi disabled.', file=sys.stderr)
                    self.cropper.params.crop_enabled = False

        # Do lots of awesome groundbreaking robot stuff here
        result = ObjectRecognitionResult()
        self.plasm.execute(niter=1)
        # the pipeline should have published, wait for the results.
        while self.recognition_result is None:  # self.poses is None or self.object_ids is None:
            rospy.loginfo('ORK results: waiting')
            rospy.sleep(0.1)
        result.recognized_objects = self.recognition_result
        # we have a result!
        self.server.set_succeeded(result=result)

        # reset our instance variable for the next round
        self.recognition_result = None
        rospy.loginfo('ORK results: received')
