#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Example Python node to listen on a specific topic.
"""

from __future__ import division
# Import required Python code.
import rospy
import time
import numpy as np
from mavros_msgs.msg import State
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import mavros 
import subprocess


def command_path_xy(start_setpoint, end_setpoint, speed_mps=1.0):
    """
    Setpoints as (x, y) tuples to be sent to /?_rel_setpoint parameter
    """
    path_length = np.sqrt((start_setpoint[0] - end_setpoint[0])**2 +
                          (start_setpoint[1] - end_setpoint[1])**2)
    duration_s = path_length / speed_mps

    # Create interpolated position arrays with 10 * duration(seconds) points
    x_array = np.linspace(start_setpoint[0], end_setpoint[0], duration_s*10.0);
    y_array = np.linspace(start_setpoint[1], end_setpoint[1], duration_s*10.0);
    
    for i in range(len(x_array)):
        # Apply setpoint parameters at 10 Hz
        #listener()
        rospy.set_param('/x_rel_setpoint', float(x_array[i])) 
        rospy.set_param('/y_rel_setpoint', float(y_array[i]))
        if rospy.get_param('/tag_detect') == 1:
            print "Found April Tag!!"
            land_now()
        time.sleep(0.1)

def land_now():
    print "Landing Now"
    rospy.set_param('/land_now', 1)
    time.sleep(2.5)
    print "Setting zero velocity target"
    rospy.set_param('/zero_vel', 1)


def cone_search():
    '''
    Main function.
    '''

    while not rospy.is_shutdown(): 
        time.sleep(0.5)
        if rospy.get_param('/offboard') < 1:
            print "Still Waiting for Offboard"
        else:
            break

    # Square-shaped cone search path
    xy_setpoints = [( 0.0,  0.0),
                    (-0.6,  0.0),
                    (-0.6, -0.6),
                    ( 0.6, -0.6),
                    ( 0.6, -1.2),
                    ( 0.0, -1.2)]

    # Visit each setpoint
    for i in range(len(xy_setpoints)-1):
        rospy.loginfo("Setting new waypoint: %f, %f", xy_setpoints[i+1][0],
                                                      xy_setpoints[i+1][1]) 
        command_path_xy(xy_setpoints[i], xy_setpoints[i+1], speed_mps=0.2)
        time.sleep(2)
    land_now()    



# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('cone_script')
    # Go to the main loop.
    # /S# with correct custom mode we can wait until it changes
    # state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
    cone_search()
