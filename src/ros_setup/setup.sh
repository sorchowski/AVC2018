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

#check if setup.bash for both the ros distro and the ros workspace have run
if [[ -z ${ROS_PACKAGE_PATH} ]]; then
    echo 'Did you run setup.bash for your ros distro?'
    exit 1;
else
    if [[ ${ROS_PACKAGE_PATH} = *"${ROS_WORKSPACE_HOME}"* ]]; then
        echo "its there!"
    else
        echo 'Did you run setup.bash for your ros workspace?'
        exit 1;
    fi
fi

#create 'src' and 'include' directories under <ros workspace>/src/avc/ if necessary

if [ ! -d "$ROS_WORKSPACE_HOME/src/$ROS_AVC_PROJECT_NAME" ]; then
    echo 'avc package does not exist, creating'
    cd $ROS_WORKSPACE_HOME/src/
    catkin_create_pkg $ROS_AVC_PROJECT_NAME
    mkdir -p $ROS_WORKSPACE_HOME/src/$ROS_AVC_PROJECT_NAME/include
    mkdir -p $ROS_WORKSPACE_HOME/src/$ROS_AVC_PROJECT_NAME/src
    mkdir -p $ROS_WORKSPACE_HOME/src/$ROS_AVC_PROJECT_NAME/param
    cd -
fi

#copy CMakeLists.txt and package.xml to <ros workspace/src/avc/
echo 'Copying avc package setup files'
cp CMakeLists.txt $ROS_WORKSPACE_HOME/src/$ROS_AVC_PROJECT_NAME/
cp package.xml $ROS_WORKSPACE_HOME/src/$ROS_AVC_PROJECT_NAME/
cp avc.launch $ROS_WORKSPACE_HOME/src/$ROS_AVC_PROJECT_NAME/
cp avc_nav.launch $ROS_WORKSPACE_HOME/src/$ROS_AVC_PROJECT_NAME/

#copy our own modules to <ros_workspace>/src/avc/src
echo 'Copying avc source files'
cp ../modules/gps/node_read_gps.cpp $ROS_WORKSPACE_HOME/src/$ROS_AVC_PROJECT_NAME/src/
cp ../modules/display/display.cpp $ROS_WORKSPACE_HOME/src/$ROS_AVC_PROJECT_NAME/src/
cp ../modules/convert/node_quad_convert.cpp $ROS_WORKSPACE_HOME/src/$ROS_AVC_PROJECT_NAME/src/
cp ../modules/convert/node_range_convert.cpp $ROS_WORKSPACE_HOME/src/$ROS_AVC_PROJECT_NAME/src/
cp ../modules/convert/node_imu_convert.cpp $ROS_WORKSPACE_HOME/src/$ROS_AVC_PROJECT_NAME/src/
cp ../modules/nav/*.yaml $ROS_WORKSPACE_HOME/src/$ROS_AVC_PROJECT_NAME/param/

#copy AVC/lib/common/*.* to <ros_workspace>/src/avc/include
cp ../../lib/common/*.* $ROS_WORKSPACE_HOME/src/$ROS_AVC_PROJECT_NAME/include

#copy the rules files for node devices
echo 'Copying node rules file to /etc/udev/rules.d'
sudo cp ros_nodes.rules /etc/udev/rules.d/

#in top-level ros workspace directory, enter "catkin_make"
echo "Go to $ROS_WORKSPACE_HOME/, and enter 'catkin_make'"

#To install new package to system,
# TODO: check whether this should be installed in bin or lib
# $ cd ~/catkin_ws
# $ catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/kinetic  # might need sudo

