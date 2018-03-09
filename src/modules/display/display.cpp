#include "ros/ros.h"
#include "ros_topics.h"

#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Range.h"

struct avc_data_t {
	float sonar1;
	float sonar2;
	float sonar3;
	float sonar4;
	float sonar5;
	float sonar6;
	float sonar7;

	float ir1;
	float ir2;
	float ir3;

	float velocity;
	
	float mag_x;
	float mag_y;
	float mag_z;

	float imu_acc_x;
	float imu_acc_y;
	float imu_acc_z;
	float imu_ang_x;
	float imu_ang_y;
	float imu_ang_z;

	float latitude;
	float longitude;
	float altitude;

} avc_data;

void sonarCallback(const sensor_msgs::Range& range_message)
{
	char * frame_id = range_message.header.frame_id;

	if (("/sonar1".compare(frame_id) == 0) {
		avc_data.sonar1 = range_message.range;
	} else if (("/sonar2".compare(frame_id) == 0) {
		avc_data.sonar2 = range_message.range;
	} else if (("/sonar3".compare(frame_id) == 0) {
		avc_data.sonar3 = range_message.range;
	} else if (("/sonar4".compare(frame_id) == 0) {
		avc_data.sonar4 = range_message.range;
	} else if (("/sonar5".compare(frame_id) == 0) {
		avc_data.sonar5 = range_message.range;
	} else if (("/sonar6".compare(frame_id) == 0) {
		avc_data.sonar6 = range_message.range;
	} else if (("/sonar7".compare(frame_id) == 0) {
		avc_data.sonar7 = range_message.range;
	}
}

void odometerCallback(const geometry_msgs::TwistStamped& odom_message)
{
	avc_data.velocity = odom_message.twist.linear.x;
}

void imuCallback(sensor_msgs::Imu& imu_message)
{
	avc_data.imu_acc_x = imu_message.linear_acceleration.x;
	avc_data.imu_acc_y = imu_message.linear_acceleration.y;
	avc_data.imu_acc_z = imu_message.linear_acceleration.z;
	avc_data.imu_ang_x = imu_message.angular_velocity.x;
	avc_data.imu_ang_y = imu_message.angular_velocity.y;
	avc_data.imu_ang_z = imu_message.angular_velocity.z;
}

void irCallback(const sensor_msgs::Range& range_message)
{
	char * frame_id = range_message.header.frame_id;

	if (("/ir1".compare(frame_id) == 0) {
		avc_data.ir1 = range_message.range;
	} else if (("/ir2".compare(frame_id) == 0) {
		avc_data.ir2 = range_message.range;
	} else if (("/ir3".compare(frame_id) == 0) {
		avc_data.ir3 = range_message.range;
	}
}

void gpsCallback(sensor_msgs::NavSatFix& navsatfix_msg)
{
	avc_data.latitude = navsatfix_msg.latitude;
	avc_data.longitude = navsatfix_msg.longitude;
	avc_data.altitude = navsatfix_msg.altitude;
}

void magCallback(sensor_msgs::MagneticField& mag_message)
{
	avc_data.mag_x = mag_message.magnetic_field.x;
	avc_data.mag_y = mag_message.magnetic_field.y;
	avc_data.mag_z = mag_message.magnetic_field.z;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "display");

	ros::NodeHandle nh;

	ros::Subscriber odometrySub = nh.subscribe(avc_common::ROS_TOPIC_ODOMETRY, 1000, odometerCallback);
	ros::Subscriber imuSub = nh.subscribe(avc_common::ROS_TOPIC_IMU, 1000, imuCallback);
	ros::Subscriber irSub = nh.subscribe(avc_common::ROS_TOPIC_INFRARED, 1000, irCallback);
	ros::Subscriber sonarSub = nh.subscribe(avc_common::ROS_TOPIC_SONAR, 1000, sonarCallback);
	ros::Subscriber gpsSub = nh.subscribe(avc_common::ROS_TOPIC_GPS, 1000, gpsCallback);
	ros::Subscriber magSub = nh.subscribe(avc_common::ROS_TOPIC_MAG, 1000, magCallback);

	while(ros::ok()) {
		//ros::spin();
		// TODO: print data values if timer ok
		ros::spinOnce();
	}

	return 0;
}
