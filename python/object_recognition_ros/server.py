#!/usr/bin/env python
"""
Define an actionlib server for ORK
"""
from object_recognition_core.pipelines.plasm import create_plasm
from object_recognition_msgs.msg import RecognizedObjectArray
from object_recognition_msgs.msg._ObjectRecognitionAction import ObjectRecognitionAction
from object_recognition_msgs.msg._ObjectRecognitionResult import ObjectRecognitionResult
import actionlib
import rospy
import sys

class RecognitionServer:
    """
    Main server that reads a config file, builds an actionlib server, reads an ecto plasm and run it when
    the actionlib server is queried
    """
    recognition_result = None
    cropper = None

    def __init__(self, ork_params):
        # create the plasm that will run the detection
        self.plasm = create_plasm(ork_params)
        self.plasm.configure_all()
        print 'configured'

        # the results or the object recognition pipeline
        self.recognition_result = None

        topics = ['recognized_object_array']
        
        for sink in ork_params.itervalues():
            if 'recognized_object_array_topic' in sink:
                topics.append(sink['recognized_object_array_topic'])

        # subscribe to the output of the detection pipeline
        for topic in topics:
            rospy.Subscriber(topic, RecognizedObjectArray, self.callback_recognized_object_array)
            print 'Subscribed to the ' + topic + ' topic.'

        # look for a cell that contains a cropper to select the ROI
        for cell in self.plasm.cells():
            if 'crop_enabled' in cell.params:
                self.cropper = cell

        # actionlib stuff
        self.server = actionlib.SimpleActionServer('recognize_objects', ObjectRecognitionAction, self.execute, False)
        self.server.start()
        print 'started'

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
                    print >> sys.stderr, 'WARNING: goal.use_roi is enabled but filter_limits doesn\'t have size 6 [x_min, x_max, y_min, y_max, z_min, z_max]. Roi disabled.'
                    self.cropper.params.crop_enabled = False

        # Do lots of awesome groundbreaking robot stuff here
        result = ObjectRecognitionResult()
        self.plasm.execute(niter=1)
        # the pipeline should have published, wait for the results.
        while self.recognition_result is None:  # self.poses is None or self.object_ids is None:
            print 'waiting'
            rospy.sleep(0.1)
        result.recognized_objects = self.recognition_result
        # we have a result!
        self.server.set_succeeded(result=result)

        # reset our instance variable for the next round
        self.recognition_result = None
