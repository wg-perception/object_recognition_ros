#!/usr/bin/env python
from object_recognition_msgs.msg import ObjectRecognitionAction, ObjectRecognitionGoal
import actionlib
import argparse
import rospy
import subprocess
import sys

def on_result(status, result):
    rospy.loginfo('Received result from ORK.')

if __name__ == '__main__':
    rospy.init_node('recognition_client')

    proc = subprocess.Popen(['./test_server2.py'], shell=True)

    client = actionlib.SimpleActionClient('recognize_objects', ObjectRecognitionAction)
    client.wait_for_server()

    start = rospy.Time.now()  # for checking the round trip time.

    goal = ObjectRecognitionGoal()

    # Sample region of interest for object detection (disabled by default)
    # goal.use_roi = True
    # goal.filter_limits = [-0.4, 0.4, -1.0, 0.2, 0.01, 1.5]

    client.send_goal(goal, done_cb=on_result)
    client.wait_for_result()  # wait indefinitely for a result

    # print out the round trip time.
    rospy.loginfo('Time for 1 detection: %s', (rospy.Time.now() - start).to_sec())
