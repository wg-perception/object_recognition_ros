#!/usr/bin/env python
"""
This file is a client to a server: it keeps making request and prints the output
from the ORK actionlib server
"""
import actionlib
import argparse
import rospy
import sys
from object_recognition_msgs.msg import ObjectRecognitionAction, ObjectRecognitionGoal

def on_result(status, result):
    print result

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Client that queries the ORK server and prints the output. '
                                     'Start your server and launch that file for testing.')
    args = parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node('recognition_client')
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
    print "Time for 1 detection:", (rospy.Time.now() - start).to_sec()
