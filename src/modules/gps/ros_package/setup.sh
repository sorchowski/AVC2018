# Figure out ros workspace top-level directory

# under <ros workspace>/src/, enter "catkin_create_pkg avc"

#create 'src' and 'include' directories under <ros workspace>/src/avc/

#copy CMakeLists.txt and package.xml to <ros workspace/src/avc/

#copy node_read_gps.cpp to <ros_workspace>/src/avc/src

#copy AVC/lib/common/*.* to <ros_workspace>/src/avc/include

#in top-level ros workspace directory, enter "catkin_make"



#To install new package to system,
# TODO: check whether this should be installed in bin or lib
# $ cd ~/catkin_ws
# $ catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/kinetic  # might need sudo
