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
import tf
import math
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

def get_yaw(pose):
    quaternion = (
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion, axes='szyx')
    yaw = euler[0]
    return yaw
    
def april_cb(msg):
    #rospy.loginfo("Got reading")
    rospy.set_param('/filtered_detect', 1)
    # Saving all values in X right, Y forwards frame
    rospy.loginfo("###### NEW FILTERED TAG DETECT ######")
    rospy.loginfo("Tag location xyz: {} {} {}".format(msg.pose.pose.position.x,
                                                                   msg.pose.pose.position.y,
                                                                   msg.pose.pose.position.z)) 
    rospy.set_param('/filtered_tag_x', msg.pose.pose.position.x)
    rospy.set_param('/filtered_tag_y', msg.pose.pose.position.y)
    rospy.set_param('/filtered_tag_z', msg.pose.pose.position.z)
    rospy.set_param('/filtered_tag_yaw', msg.pose.pose.orientation.z) # yaw
    rospy.set_param('/pose_last_tagupdate_x', -current_pose.pose.position.x)
    rospy.set_param('/pose_last_tagupdate_y', -current_pose.pose.position.y)
    rospy.set_param('/pose_last_tagupdate_z', current_pose.pose.position.z)
    rospy.set_param('/pose_last_tagupdate_yaw', get_yaw(current_pose.pose))
    rospy.set_param('/pose_last_tagupdate_time', msg.header.stamp.secs)

current_pose = None # Global variable to hold the current pose
def pose_cb(msg):
    global current_pose
    current_pose = msg

def tag_detect_cb(msg):
    #rospy.loginfo("Tag Detected")
    rospy.set_param('/tag_detect', 1)
#    rospy.set_param('/pose_last_tagupdate_x', -current_pose.pose.position.x)
#    rospy.set_param('/pose_last_tagupdate_y', -current_pose.pose.position.y)
#    rospy.set_param('/pose_last_tagupdate_z', current_pose.pose.position.z)
#    rospy.set_param('/pose_last_tagupdate_yaw', get_yaw(current_pose.pose))
#    rospy.set_param('/pose_last_tagupdate_time', msg.header.stamp.secs)

def listener():
    rospy.init_node('tag_listener', anonymous=True)
    rospy.Subscriber('filtered_pose', PoseWithCovarianceStamped, april_cb)
    rospy.Subscriber('mavros/local_position/local', PoseStamped, pose_cb)
    rospy.Subscriber('rectified_pose', PoseStamped, tag_detect_cb)
    rospy.spin()

if __name__ == '__main__':
    listener()
