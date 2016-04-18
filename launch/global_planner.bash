#!/bin/bash
roscore &
sleep 1
# Record rosbags, excluding video
rosbag record -a -x "/camera/image(.*)|/april_tag_debug/(.*)" &

cd ~/catkin_ws/
source devel/setup.bash
source ./src/gscam_config.sh

cd ~/column_ws/
source devel/setup.bash

rosrun gscam gscam &
sleep 1

roslaunch mavros px4.launch &
sleep 1

roslaunch column search_hover_land.launch
 
