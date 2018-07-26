#include "ros/ros.h"
#include <sensor_msgs/Range.h>
// AVC includes
#include "ros_topics.h"
#include "node_names.h"

#define SAMPLE_RATE 100

/**
 * The purpose of this class is to merge range messages from Infrared and Sonar sensors into a single
 * point-cloud message for processing by the ros navigation stack.
 */
class RangeConverter {

  public:
    RangeConverter() {

    }

    void irMsgCallback(const sensor_msgs::Range& msg)
    {
      //ROS_INFO("I heard: [%s]", msg->data.c_str());
      ROS_INFO("I HEARD A IR Range MSG");
      printf("SEO: I heard a range msg");

      // TODO: add this to the pc message
    }

    void sonarMsgCallback(const sensor_msgs::Range& msg) {
      ROS_INFO("I HEARD A SONAR RANGE MSG");
      prinft("SEO: I heard a sonar range msg");
    }

  private:
    ros::NodeHandle n;

    // publisher for point cloud messages
    ros::Publisher pc_pub = n.advertise<>("????", 1000);

    // subscriber for Range messages from the avc ir sensors.
    ros::Subscriber ir_sub = n.subscribe(avc_common::ROS_TOPIC_INFRARED, 1000, irMsgCallback);

    // subscriber for Range messages from the avc sonar sensors.
    ros::Subscriber sonar_sub = n.subscribe(avc_common::ROS_TOPIC_SONAR, 1000, sonarMsgCallback);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, avc_common::NODE_NAME_CONVERT_RANGE);

  RangeConverter rangeConverter;

  //TODO test both of these
  //ros::spin();
  while(true) { ros::Rate(SAMPLE_RATE).sleep(); ros::spinOnce(); }

  return 0;
}