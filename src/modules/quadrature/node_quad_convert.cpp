#include "ros/ros.h"

// AVC includes
#include "ros_topics.h"
#include "device_paths.h"
#include "node_names.h"

void quadCallback(const geometry_msgs::TwistStamped& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, avc_common::NODE_NAME_CONVERT_QUADRATURE);

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe(avc_common::ROS_TOPIC_ODOMETRY, 1000, quadCallback);

  ros::spin();

  return 0;
}