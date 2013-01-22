from __future__ import print_function
from object_recognition_ros.server import DEFAULT_NODE_NAME
import ecto_ros
import rospy
import subprocess
import sys

def init_ros():
    """
    Function that should be called before starting any cell depending on ROS
    """
    # check if roscore is running
    proc = subprocess.Popen(['rostopic', 'list'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    _out, err = proc.communicate()
    if err:
        print('roscore not started: start it or your ROS cell will error out at runtime')
    else:
        ecto_ros.init(sys.argv, DEFAULT_NODE_NAME, False)
