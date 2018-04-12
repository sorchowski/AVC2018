#!/bin/bash

ROS_WORKSPACE_HOME=""
ROS_AVC_PROJECT_NAME="avc"

# Figure out ros workspace top-level directory
if [ "$1" == "" ]; then
    echo "Need to pass in ros workspace parameter";
    exit 1;
else
    ROS_WORKSPACE_HOME=$1
fi

echo "target ros workspace:" $ROS_WORKSPACE_HOME

if [ ! -d "$ROS_WORKSPACE_HOME/src" ]; then
    echo "$ROS_WORKSPACE_HOME/src does not exist";
    exit 1;
fi

#create 'src' and 'include' directories under <ros workspace>/src/avc/ if necessary

if [ ! -d "$ROS_WORKSPACE_HOME/src/$ROS_AVC_PROJECT_NAME" ]; then
    echo 'avc package does not exist, creating'
    cd $ROS_WORKSPACE_HOME/src/
    catkin_create_pkg $ROS_AVC_PROJECT_NAME
    mkdir -p $ROS_WORKSPACE_HOME/src/$ROS_AVC_PROJECT_NAME/include
    mkdir -p $ROS_WORKSPACE_HOME/src/$ROS_AVC_PROJECT_NAME/src
    cd -
fi

#copy CMakeLists.txt and package.xml to <ros workspace/src/avc/
echo 'Copying ros make files'
cp CMakeLists.txt $ROS_WORKSPACE_HOME/src/$ROS_AVC_PROJECT_NAME/
cp package.xml $ROS_WORKSPACE_HOME/src/$ROS_AVC_PROJECT_NAME/

#copy our own modules to <ros_workspace>/src/avc/src
echo 'Copying avc source files'
cp ../modules/gps/node_read_gps.cpp $ROS_WORKSPACE_HOME/src/$ROS_AVC_PROJECT_NAME/src
cp ../modules/display/display.cpp $ROS_WORKSPACE_HOME/src/$ROS_AVC_PROJECT_NAME/src

#copy AVC/lib/common/*.* to <ros_workspace>/src/avc/include
cp ../../lib/common/*.* $ROS_WORKSPACE_HOME/src/$ROS_AVC_PROJECT_NAME/include

#in top-level ros workspace directory, enter "catkin_make"
echo "Go to $ROS_WORKSPACE_HOME/src/, and enter 'catkin_make'"

#To install new package to system,
# TODO: check whether this should be installed in bin or lib
# $ cd ~/catkin_ws
# $ catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/kinetic  # might need sudo

