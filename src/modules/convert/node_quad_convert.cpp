#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"

// AVC includes
#include "ros_topics.h"
#include "node_names.h"

class QuadratureConverter {

  public:
    QuadratureConverter() {

    }

    /**
      * Callback for receiving Twist messages from the odometry sensors.
      * We will convert this message into an official ROS Odometry message.
      */
    void quadCallback(const geometry_msgs::TwistStamped& msg)
    {
      //ROS_INFO("I heard: [%s]", msg->data.c_str());
      ROS_INFO("I HEARD A QUAD MSG");
      printf("SEO: I heard a quad msg");

      // TODO: convert Twist msg to Odometry msg

      // TODO: odometry_pub.publish(odometry_msg);
    }

  private:
    ros::NodeHandle n;

    // publisher for Odometry messages
    ros::Publisher odometry_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);

    // subscriber for Twist messages from the avc odometry sensors.
    ros::Subscriber sub = n.subscribe(avc_common::ROS_TOPIC_ODOMETRY, 1000, quadCallback);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, avc_common::NODE_NAME_CONVERT_QUADRATURE);

  QuadratureConverter quadratureConverter;

  //TODO test both of these
  //ros::spin();
  while(true) { ros::Rate(100).sleep(); ros::spinOnce(); }

  return 0;
}