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
    rospy.set_param('/filtered_tag_x', msg.pose.pose.position.x)
    rospy.set_param('/filtered_tag_y', msg.pose.pose.position.y)
    rospy.set_param('/filtered_tag_y', msg.pose.pose.position.y)
    rospy.set_param('/pose_last_tagupdate_x', 0.0)  # TODO: Actually update and use this
    rospy.set_param('/pose_last_tagupdate_y', 0.0)
    rospy.set_param('/pose_last_tagupdate_z', 0.0)

def listener():
    rospy.init_node('tag_listener', anonymous=True)
    rospy.Subscriber("filtered_pose", PoseWithCovarianceStamped, april_cb)
    rospy.spin()

if __name__ == '__main__':
    listener()
