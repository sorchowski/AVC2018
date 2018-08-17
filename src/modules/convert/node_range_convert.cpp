#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>

// AVC includes
#include "ros_topics.h"
#include "node_names.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// twice a second
#define PC_PUBLISH_RATE 0.5

/**
 * The purpose of this class is to merge range messages from Infrared and Sonar sensors into a single
 * point-cloud message for processing by the ros navigation stack.
 */
class RangeConverter {

  public:

    RangeConverter() {
      pc_pub = n.advertise<PointCloud>("points2", 1000);
      range_sub = n.subscribe(avc_common::ROS_TOPIC_RANGE, 1000, &RangeConverter::rangeMsgCallback, this);
      previous_time = ros::Time::now();
    }

    void rangeMsgCallback(const sensor_msgs::Range& r_msg)
    {
      std::string source_frame_id = r_msg.header.frame_id;

      try {      
        geometry_msgs::PointStamped range_point;
        range_point.header.frame_id = r_msg.header.frame_id;
        range_point.point.x = r_msg.range;
        geometry_msgs::PointStamped base_point;

        // if the range message has a range value outside the min/max range,
        // the point value associated with this range should be cleared.

        if (r_msg.range > r_msg.max_range || r_msg.range < r_msg.min_range) {
          // Clear the point if there exist one for the frame id
          if (points_map.find(source_frame_id) != points_map.end()) {
            points_map.erase(source_frame_id);
          }
        } else {

          // base_link is always our target frame
          listener.transformPoint("base_link", range_point, base_point);

          float x = base_point.point.x;
          float y = base_point.point.y;
          float z = base_point.point.z;

          points_map[source_frame_id] = pcl::PointXYZ(x, y, z);
        }

        // If it's time to publish the point cloud, do so
        ros::Time current_time = ros::Time::now();
        ros::Duration time_diff = current_time-previous_time;
        if (time_diff > ros::Duration(PC_PUBLISH_RATE)) {

          PointCloud::Ptr pc_msg (new PointCloud);
          pc_msg->header.frame_id = "base_link";
          pc_msg->height = 1;

          // iterate through the points map to build a new point cloud
          std::map<std::string, pcl::PointXYZ>::iterator it = points_map.begin();
          int width = 0;
          while (it != points_map.end()) {
            pcl::PointXYZ curr_pt = it->second;
            pc_msg->points.push_back(pcl::PointXYZ(curr_pt.x, curr_pt.y, curr_pt.z));
            width++;
            it++;
          }
          pc_msg->width = width;
          pcl_conversions::toPCL(r_msg.header.stamp, pc_msg->header.stamp);
          pc_pub.publish(pc_msg);

          previous_time = current_time;
        }
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("Error attempting to transform range msg: %s", ex.what());
      }
    }

  private:

    ros::NodeHandle n;

    // publisher for point cloud messages
    ros::Publisher pc_pub;

    // subscriber for Range messages from the avc ir sensors.
    ros::Subscriber range_sub;

    tf::TransformListener listener;

    // Contains a mapping of sensor frames to points
    std::map<std::string, pcl::PointXYZ> points_map;

    ros::Time previous_time;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, avc_common::NODE_NAME_CONVERT_RANGE);

  RangeConverter rangeConverter;

  //TODO test both of these
  ros::spin();

  return 0;
}
