#!/bin/bash
#roscore &
#sleep 1
# Record rosbags, excluding video
rosbag record -a -x "/camera/(.*)" &

# Add GScam ros env and config
#source ~/Development/ROS-semi-official/devel/setup.bash
#source ~/catkin_ws/src/column/gscam_config.sh

roslaunch openni2_launch openni2.launch depth_registration:=true &
sleep 1

cd ~/catkin_ws/
source devel/setup.bash

#roslaunch mavros px4.launch &
#sleep 1

roslaunch column move_base.launch
