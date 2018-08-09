#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>

// AVC includes
#include "ros_topics.h"
#include "node_names.h"

#define SAMPLE_RATE 20

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/**
 * The purpose of this class is to merge range messages from Infrared and Sonar sensors into a single
 * point-cloud message for processing by the ros navigation stack.
 */
class RangeConverter {

  public:

    RangeConverter() {
      pc_pub = n.advertise<PointCloud>("points2", 1000);
      range_sub = n.subscribe(avc_common::ROS_TOPIC_RANGE, 1000, &RangeConverter::rangeMsgCallback, this);
    }

    void rangeMsgCallback(const sensor_msgs::Range& r_msg)
    {
      //ROS_INFO("I heard: [%s]", msg->data.c_str());

      // For testing, just look at the left sonar sensor rotated 90
      if (r_msg.header.frame_id=="s1") {
        geometry_msgs::PointStamped range_point;
        range_point.header.frame_id = r_msg.header.frame_id;
        range_point.point.x = r_msg.range;
        geometry_msgs::PointStamped base_point;
        listener.transformPoint("base_link", range_point, base_point);

        float x = base_point.point.x;
        float y = base_point.point.y;
        float z = base_point.point.z;
        //http://wiki.ros.org/pcl_ros#ROS_C.2B-.2B-_interface
        PointCloud::Ptr pc_msg (new PointCloud);
        pc_msg->header.frame_id = "base_link";
        pc_msg->height = pc_msg->width = 1;
        pc_msg->points.push_back (pcl::PointXYZ(x, y, z));
        pcl_conversions::toPCL(r_msg.header.stamp, pc_msg->header.stamp);

        pc_pub.publish(pc_msg);
      }
    }

  private:

    ros::NodeHandle n;

    // publisher for point cloud messages
    ros::Publisher pc_pub;

    // subscriber for Range messages from the avc ir sensors.
    ros::Subscriber range_sub;

    tf::TransformListener listener;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, avc_common::NODE_NAME_CONVERT_RANGE);

  RangeConverter rangeConverter;

  //TODO test both of these
  //ros::spin();
  while(true) { ros::Rate(SAMPLE_RATE).sleep(); ros::spinOnce(); }

  return 0;
}
