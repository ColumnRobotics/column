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
from geometry_msgs.msg import PoseWithCovarianceStamped

def april_cb(msg):
    rospy.loginfo("Tag Detected")
    rospy.set_param('/tag_detect', 1)
    # Saving all values in X right, Y forwards frame
    rospy.set_param('/filtered_tag_x', -1.0 * msg.pose.pose.position.x)
    rospy.set_param('/filtered_tag_y', msg.pose.pose.position.y)

def listener():
    rospy.init_node('tag_listener', anonymous=True)
    rospy.Subscriber("filtered_pose", PoseWithCovarianceStamped, april_cb)
    rospy.spin()

if __name__ == '__main__':
    listener()
