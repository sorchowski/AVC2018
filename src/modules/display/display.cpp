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
	unsigned int sonar_count;

	float ir1;
	float ir2;
	float ir3;
	unsigned int ir_count;

	float velocity;
	unsigned int velocity_count;
	
	float mag_x;
	float mag_y;
	float mag_z;
	unsigned int mag_count;

	float imu_acc_x;
	float imu_acc_y;
	float imu_acc_z;
	float imu_ang_x;
	float imu_ang_y;
	float imu_ang_z;
	unsigned int imu_count;

	float latitude;
	float longitude;
	float altitude;
	unsigned int gps_count;
} avc_data;

void sonarCallback(const sensor_msgs::Range::ConstPtr& range_message)
{
	avc_data.sonar_count += 1;

	const char * frame_id = range_message->header.frame_id.c_str();

	if (strcmp("/sonar1", frame_id) == 0) {
		avc_data.sonar1 = range_message->range;
	} else if (strcmp("/sonar2", frame_id) == 0) {
		avc_data.sonar2 = range_message->range;
	} else if (strcmp("/sonar3", frame_id) == 0) {
		avc_data.sonar3 = range_message->range;
	} else if (strcmp("/sonar4", frame_id) == 0) {
		avc_data.sonar4 = range_message->range;
	} else if (strcmp("/sonar5", frame_id) == 0) {
		avc_data.sonar5 = range_message->range;
	} else if (strcmp("/sonar6", frame_id) == 0) {
		avc_data.sonar6 = range_message->range;
	} else if (strcmp("/sonar7", frame_id) == 0) {
		avc_data.sonar7 = range_message->range;
	}
}

void odometerCallback(const geometry_msgs::TwistStamped::ConstPtr& odom_message)
{
	avc_data.velocity_count += 1;

	avc_data.velocity = odom_message->twist.linear.x;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_message)
{
	avc_data.imu_count += 1;

	avc_data.imu_acc_x = imu_message->linear_acceleration.x;
	avc_data.imu_acc_y = imu_message->linear_acceleration.y;
	avc_data.imu_acc_z = imu_message->linear_acceleration.z;
	avc_data.imu_ang_x = imu_message->angular_velocity.x;
	avc_data.imu_ang_y = imu_message->angular_velocity.y;
	avc_data.imu_ang_z = imu_message->angular_velocity.z;
}

void irCallback(const sensor_msgs::Range::ConstPtr& range_message)
{
	avc_data.ir_count += 1;

	const char * frame_id = range_message->header.frame_id.c_str();

	if (strcmp("/ir1", frame_id) == 0) {
		avc_data.ir1 = range_message->range;
	} else if (strcmp("/ir2", frame_id) == 0) {
		avc_data.ir2 = range_message->range;
	} else if (strcmp("/ir3", frame_id) == 0) {
		avc_data.ir3 = range_message->range;
	}
}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& navsatfix_msg)
{
	avc_data.gps_count += 1;

	avc_data.latitude = navsatfix_msg->latitude;
	avc_data.longitude = navsatfix_msg->longitude;
	avc_data.altitude = navsatfix_msg->altitude;
}

void magCallback(const sensor_msgs::MagneticField::ConstPtr& mag_message)
{
	avc_data.mag_count += 1;

	avc_data.mag_x = mag_message->magnetic_field.x;
	avc_data.mag_y = mag_message->magnetic_field.y;
	avc_data.mag_z = mag_message->magnetic_field.z;
}

void printData()
{
	printf("\33c\e[3J");
	printf("sonar: #%-5d 1:%2.3f 2:%2.3f 3:%2.3f 4:%2.3f 5:%2.3f 6:%2.3f 7:%2.3f\n", avc_data.sonar_count, avc_data.sonar1, avc_data.sonar2, avc_data.sonar3, avc_data.sonar4, avc_data.sonar5, avc_data.sonar6, avc_data.sonar7);
	printf("imu acc: #%-5d x:%3.3f y:%3.3f z:%3.3f\n",avc_data.imu_count, avc_data.imu_acc_x, avc_data.imu_acc_y, avc_data.imu_acc_z);
	printf("imu ang: #%-5d x:%3.3f y:%3.3f z:%3.3f\n",avc_data.imu_count, avc_data.imu_ang_x, avc_data.imu_ang_y, avc_data.imu_ang_z);
	printf("mag: #%-5d x:%3.3f y:%3.3f z:%3.3f\n", avc_data.imu_count, avc_data.mag_x, avc_data.mag_y, avc_data.mag_z);
	printf("velocity: #%-5d - %2.3f m/s\n", avc_data.velocity_count, avc_data.velocity); 
	printf("gps: #%-5d lat: %3.3f lon: %3.3f alt: %3.3f\n", avc_data.gps_count, avc_data.latitude, avc_data.longitude, avc_data.altitude);

}

int main(int argc, char **argv)
{
	avc_data.sonar_count = 0;
	avc_data.ir_count = 0;
	avc_data.mag_count = 0;
	avc_data.imu_count = 0;
	avc_data.gps_count = 0;
	avc_data.velocity_count = 0;

	ros::init(argc, argv, "display");

	ros::NodeHandle nh;

	ros::Subscriber odometrySub = nh.subscribe(avc_common::ROS_TOPIC_ODOMETRY, 1000, odometerCallback);
	ros::Subscriber imuSub = nh.subscribe(avc_common::ROS_TOPIC_IMU, 1000, imuCallback);
	ros::Subscriber irSub = nh.subscribe(avc_common::ROS_TOPIC_INFRARED, 1000, irCallback);
	ros::Subscriber sonarSub = nh.subscribe(avc_common::ROS_TOPIC_SONAR, 1000, sonarCallback);
	ros::Subscriber gpsSub = nh.subscribe(avc_common::ROS_TOPIC_GPS, 1000, gpsCallback);
	ros::Subscriber magSub = nh.subscribe(avc_common::ROS_TOPIC_MAG, 1000, magCallback);

	ros::Rate loop_rate(2);

	while(ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
		printData();
	}

	return 0;
}
