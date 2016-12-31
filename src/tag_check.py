#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
April tag checker for offboard column node, changes the tag_detect param
"""

from __future__ import division
# Import required Python code.
import rospy
import time
import numpy as np
from mavros_msgs.msg import State
from geometry_msgs.msg import Pose

def april_cb(msg):
    print "Tag Detected"
    rospy.set_param('/tag_detect', 1)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("april_pose", Pose, april_cb)
    rospy.spin()

if __name__ == '__main__':
    listener()
