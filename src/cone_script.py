#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Example Python node to listen on a specific topic.
"""

# Import required Python code.
import rospy
import time
from mavros_msgs.msg import State
from geometry_msgs.msg import Pose
import mavros 


def cone_search():
    '''
    Main function.
    '''

    while not rospy.is_shutdown(): 
        time.sleep(0.5)
        if rospy.get_param('/offboard') < 1:
            print "Still Waiting for Offboard"
        else:
            time.sleep(3)
            print "Setpoint 1"
            rospy.set_param('/x_rel_setpoint', -0.3) 
            rospy.set_param('/y_rel_setpoint', 0) 

            time.sleep(3)
            rospy.set_param('/x_rel_setpoint', -0.3) 
            rospy.set_param('/y_rel_setpoint', -0.3) 

            time.sleep(3)
            print "Setpoint 3"
            rospy.set_param('/x_rel_setpoint', 0.3) 
            rospy.set_param('/y_rel_setpoint', -0.3) 

            time.sleep(3)
            rospy.set_param('/x_rel_setpoint', 0.3) 
            rospy.set_param('/y_rel_setpoint', -0.6) 

            time.sleep(3)
            rospy.set_param('/x_rel_setpoint', 0.0) 
            rospy.set_param('/y_rel_setpoint', -0.6) 

            time.sleep(3)
            print "Landing Now"
            rospy.set_param('/land_now', 1)
            time.sleep(2)
            rospy.set_param('/zero_vel', 1)


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('cone_script')
    # Go to the main loop.
    # /S# with correct custom mode we can wait until it changes
    # state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)

    cone_search()
