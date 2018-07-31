#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"

// AVC includes
#include "ros_topics.h"
#include "node_names.h"

#define SAMPLE_RATE 50

class QuadratureConverter {

  public:
    QuadratureConverter() {
      odometry_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
      sub = n.subscribe(avc_common::ROS_TOPIC_ODOMETRY, 1000, &QuadratureConverter::quadCallback,&quadConverter);
    }

    /**
      * Callback for receiving Twist messages from the odometry sensors.
      * We will convert this message into an official ROS Odometry message.
      */
    void quadCallback(const geometry_msgs::TwistStamped& arduino_odom_msg)
    {
      //ROS_INFO("I heard: [%s]", msg->data.c_str());
      ROS_INFO("I HEARD A QUAD MSG");
      printf("SEO: I heard a quad msg");

      nav_msgs::Odometry odom_msg;
      odom_msg.child_frame_id = "base_link";
      odom_msg.twist.twist.linear.x = arduino_odom_msg.twist.linear.x;

      odometry_pub.publish(odom_msg);
    }

  private:

    ros::NodeHandle n;

     // publisher for Odometry messages
    ros::Publisher odometry_pub;

    // subscriber for Twist messages from the avc odometry sensors.
    ros::Subscriber sub;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, avc_common::NODE_NAME_CONVERT_QUADRATURE);

  QuadratureConverter quadConverter;

  //TODO test both of these
  //ros::spin();
  while(true) { ros::Rate(SAMPLE_RATE).sleep(); ros::spinOnce(); }

  return 0;
}
