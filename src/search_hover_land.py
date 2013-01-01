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

def command_path(start_setpoint, end_setpoint, speed_mps=1.0, tag_seen=False):
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
        
        # Home in on tag then land instead, if initial tag detection occurs
        if not tag_seen and rospy.get_param('/filtered_detect') == 1: return

        rospy.set_param('/x_rel_setpoint', float(x_array[i])) 
        rospy.set_param('/y_rel_setpoint', float(y_array[i]))
        time.sleep(0.1)


def land_now():
    rospy.loginfo("Landing Now")
    rospy.set_param('/land_now', 1)
    time.sleep(2.5)
    rospy.spin() # Do nothing until node closes


class waypoint_gen():
    """Iterates through waypoints with following syntax
            for x,y in waypoint_gen:

        next(waypoint_gen) also works"""

    def __init__(self, dx=0.5, dy=0.6, max_iter=8,start_x=0.0, start_y=0.0):
        self.dx = dx
        self.dy = dy
        self.x = start_x
        self.y = start_y
        self.current_iter = 0
        self.max_iter = max_iter

    def __iter__(self):
        return self

    def __next__(self):
        return self.next
    
    def next(self): # Python 3: def __next__(self)
        self.current_iter += 1
        if self.current_iter > self.max_iter:
            raise StopIteration
        else:
            # Alternate updating X and Y (X first)
            if (self.current_iter+1) % 2 == 0:
                if self.x <= 0: self.x = self.dx
                else:           self.x = -self.dx
            
            if self.current_iter % 2 == 0:
                self.y += self.dy

            return (self.x, self.y)


def lawnmower_search(dx=0.5, dy=0.6, speed_mps=0.1, waypoint_delay_s=1.0):
    '''
    perform foward lawmower search pattern to coordinates:
        x = 0.0 +/- dx  (alternating)
        y = y + dy      (increasing)
    
    X Y Z here is RIGHT FORWARDS UP    (mavros frame X, Y is negative)
    '''

    # Abort if april tag already detected
    if rospy.get_param('/filtered_detect') == 1: return

    wp_gen = waypoint_gen(dx=dx, dy=dy, max_iter=8)

    # Visit each setpoint
    last_setpoint = (0.0, 0.0)
    for setpoint in wp_gen:
        rospy.loginfo("Setting new waypoint: %f, %f", setpoint[0],setpoint[1])
        command_path(last_setpoint, setpoint, speed_mps=speed_mps)

        if rospy.get_param('/filtered_detect') == 1:  return
        last_setpoint = setpoint
        time.sleep(waypoint_delay_s)

def center_on_dock(interpolate=False, last_center=None):
    #update_time = rospy.get_param('/pose_last_tagupdate_time')
    if rospy.get_param('/filtered_detect') == 1: # Move to April Tag
        rospy.loginfo("Homing in on April Tag!!")

        # Acqire and calculate new desired setpoints from tag
        new_rel_setpoint_x = (rospy.get_param('/pose_last_tagupdate_x') 
                              - rospy.get_param('/filtered_tag_x')
                              - rospy.get_param('/x_init'))
        new_rel_setpoint_y = (rospy.get_param('/pose_last_tagupdate_y')
                              - rospy.get_param('/filtered_tag_y')
                              - rospy.get_param('/y_init'))
        new_rel_setpoint_yaw = 0.0*(rospy.get_param('/pose_last_tagupdate_yaw') 
                                    + rospy.get_param('/filtered_tag_yaw')
                                    - rospy.get_param('yaw_init'))

        # Actually command the path, without aborting on tag sightings
        if interpolate:
            current_setpoint = (rospy.get_param('/x_rel_setpoint'), 
                                rospy.get_param('/y_rel_setpoint'))
            command_path(current_setpoint, (new_rel_setpoint_x, new_rel_setpoint_y),
                         speed_mps=0.2, tag_seen=True)
        elif last_center is not None:  # Leaky integrator over three setpoints
            rospy.set_param('/x_rel_setpoint', (new_rel_setpoint_x + last_center[0] * 2) / 3)
            rospy.set_param('/y_rel_setpoint', (new_rel_setpoint_y + last_center[1] * 2) / 3)
            rospy.set_param('/yaw_rel_setpoint', new_rel_setpoint_yaw)
        else:
            rospy.set_param('/x_rel_setpoint', new_rel_setpoint_x)
            rospy.set_param('/y_rel_setpoint', new_rel_setpoint_y)
            rospy.set_param('/yaw_rel_setpoint', new_rel_setpoint_yaw)

        return new_rel_setpoint_x, new_rel_setpoint_y



if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('cone_script')
    # Go to the main loop.
    # /S# with correct custom mode we can wait until it changes
    # state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
    while not rospy.is_shutdown(): 
        time.sleep(0.5)
        if rospy.get_param('/offboard') < 1:
            rospy.loginfo("Still Waiting for Offboard")
        else:
            break
    rospy.set_param('/filtered_detect', 0)
    rospy.set_param('/tag_detect', 0)

    rospy.loginfo("Begin Search")
    lawnmower_search(dx=0.7, dy=0.6, speed_mps=0.15, waypoint_delay_s=1.0)
    # Returns when april tag has been detected
    
    time.sleep(2) # Pause 
    rospy.loginfo("Move to pre-dock")
    initial_center = center_on_dock(interpolate=True)
    rospy.set_param('/max_xy_vel', 0.2)
    #rospy.set_param('/control_gains/p', 3) # Reduce gains for more gentle movement
    #rospy.set_param('/control_gains/d', 1)
    time.sleep(2) # Pause 
    # Hold position over april tag for 5 seconds
    for _ in range(5):
        initial_center = center_on_dock(last_center=initial_center) 
        time.sleep(1)

    rospy.loginfo("Land")
    rospy.set_param('/land_now', 1)
    for _ in range(4):
        time.sleep(0.3)
        initial_center = center_on_dock(last_center=initial_center) 

    rospy.set_param('/zero_vel', 1) # Prevent suddenly rising back to target height
    rospy.set_param('/land_now', 0)
    for _ in range(5):
        time.sleep(0.3)
        initial_center = center_on_dock(last_center=initial_center) 
    
    rospy.loginfo("Final Landing Sequence")
    rospy.set_param('/land_z_vel', 0.5)
    rospy.set_param('/land_now', 1)
    rospy.set_param('/zero_vel', 0)
    while not rospy.is_shutdown():
        time.sleep(0.3)
        center_on_dock() 

