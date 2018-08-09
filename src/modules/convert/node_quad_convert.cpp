#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"


// AVC includes
#include "ros_topics.h"
#include "node_names.h"

#define SAMPLE_RATE 50

class QuadratureConverter {

  public:
    QuadratureConverter() {
      odometry_pub = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("avc_odom", 1000);
      sub = n.subscribe(avc_common::ROS_TOPIC_ODOMETRY, 1000, &QuadratureConverter::quadCallback, this);
    }

    /**
      * Callback for receiving Twist messages from the odometry sensors.
      * We will convert this message into an official ROS Odometry message.
      */
    void quadCallback(const geometry_msgs::TwistStamped& arduino_odom_msg)
    {
      geometry_msgs::TwistWithCovarianceStamped twist_msg;
      twist_msg.header.frame_id = "base_link";
      twist_msg.header.stamp = ros::Time::now();
      twist_msg.twist.twist.linear.x = arduino_odom_msg.twist.linear.x;
      twist_msg.twist.covariance[0] = 0.0001;
      odometry_pub.publish(twist_msg);
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
