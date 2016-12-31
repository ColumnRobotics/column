#!/bin/bash

roscore &
sleep 1
# Record rosbags, excluding video
rosbag record -a -x "/camera/image(.*)|/april_tag_debug/(.*)" &

cd ~/Development/ROS-semi-official
source devel/setup.bash
source ./gscam_config.sh

rosrun gscam gscam &
sleep 1

roslaunch mavros px4.launch &
sleep 1

cd ~/catkin_ws/src/april_tag
source ./devel/setup.sh

rosrun april_tag cole_node &

sleep 1

roslaunch column bt_planner.launch
 
