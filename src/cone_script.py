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

current_state = None

def state_cb(state):
    print("Current mode:", state.mode)
    current_state = state.mode


def cone_search():
    '''
    Main function.
    '''
    # offboad value: {0, 1}
    rospy.spin()
    
    # Wait for messages on topic, go to callback function when new messages
    # arrive.
    #rospy.spin()
    #ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    #        ("mavros/state", 10, state_cb);
    #ros::Subscriber tag_sub = nh.subscribe<geometry_msgs::Pose>
    #        ("april_pose", 10, tag_cb);


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('cone_script')
    # Go to the main loop.
    rospy.set_param('/x_rel_setpoint',3.0) 
    # with correct custom mode we can wait until it changes
    state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)

    cone_search()
