#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/steveo/ros_catkin_ws/devel/setup.bash

export AVC_HOME=/home/steveo/AVC
export ROS_MASTER_URI=http://192.168.1.150:11311
export ROS_IP=192.168.1.150

roslaunch avc avc.launch
