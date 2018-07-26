#include "ros/ros.h"
#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud.h>

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
    RangeConverter() {}

    void irMsgCallback(const sensor_msgs::Range& msg)
    {
      //ROS_INFO("I heard: [%s]", msg->data.c_str());
      ROS_INFO("I HEARD A IR Range MSG");
      printf("SEO: I heard a range msg");

      // TODO: add this to the pc message
    }

    void sonarMsgCallback(const sensor_msgs::Range& msg) {
      ROS_INFO("I HEARD A SONAR RANGE MSG");
      printf("SEO: I heard a sonar range msg");
    }

  private:

    // publisher for point cloud messages
    static ros::Publisher pc_pub;

    // subscriber for Range messages from the avc ir sensors.
    static ros::Subscriber ir_sub;

    // subscriber for Range messages from the avc sonar sensors.
    static ros::Subscriber sonar_sub;
};

RangeConverter rangeConverter;

ros::NodeHandle n;

ros::Publisher RangeConverter::pc_pub = n.advertise<sensor_msgs::PointCloud>("????", 1000);
ros::Subscriber RangeConverter::ir_sub = n.subscribe(avc_common::ROS_TOPIC_INFRARED, 1000, &RangeConverter::irMsgCallback, &rangeConverter);
ros::Subscriber RangeConverter::sonar_sub = n.subscribe(avc_common::ROS_TOPIC_SONAR, 1000, &RangeConverter::sonarMsgCallback, &rangeConverter);

int main(int argc, char **argv)
{
  ros::init(argc, argv, avc_common::NODE_NAME_CONVERT_RANGE);

  //TODO test both of these
  //ros::spin();
  while(true) { ros::Rate(SAMPLE_RATE).sleep(); ros::spinOnce(); }

  return 0;
}
