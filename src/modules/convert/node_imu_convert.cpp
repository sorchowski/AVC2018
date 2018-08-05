// C++ includes
#include <stdio.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdlib>
#include <cstdio>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <cstdlib>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include "ros_topics.h"
#include "node_names.h"
#include "device_paths.h"

const char imuFrameId[] = "imu";

// Way overkill, but meh...
#define READ_BUFFER_SIZE 4096  

struct termios tty;
struct termios tty_old;

int configure_usb(int ser_fd) {
	if (tcgetattr(ser_fd, &tty) != 0) {
		printf("error retrieving serial configuration\n");
		return 1;
	}

	tty_old=tty;
	// http://man7.org/linux/man-pages/man3/termios.3.html
	// c_cflag = control modes
	// c_iflag = input modes

	cfsetispeed(&tty, (speed_t)B57600);
	tty.c_cflag     &=  ~PARENB;          // Parity
	tty.c_cflag     &=  ~CSTOPB;          // two stop bits
	tty.c_cflag     &=  ~CSIZE;           // Char size
	tty.c_cflag     |=  CS8;              // char size 8
	tty.c_cflag     &=  ~CRTSCTS;         // Disable RTS/CTS (hardware flow control)
	tty.c_cc[VMIN]   =  1;                // min chars for read
	tty.c_cc[VTIME]  =  50;               // 5 seconds read timeout
	tty.c_cflag     |=  CREAD | CLOCAL;   // turn on READ & ignore ctrl lines
	tty.c_lflag     |= ICANON;            // Enable canonical mode
	tty.c_lflag     &=  ~ECHO;            // Disable echo; otherwise, the gps device will return "unknown message"
	tty.c_lflag     &=  ~ECHONL;          // Disable new line echo

	//cfmakeraw(&tty);
	tcflush(ser_fd, TCIFLUSH );
	if ( tcsetattr (ser_fd, TCSANOW, &tty) != 0) {
		printf("Error setting serial attributes\n");
		return 1;
	}

	return 0;
}

int main(int argc, char **argv) {

	// Setup the device path
	char * real_path = realpath(avc_common::ROS_NODE_DEVICE_PATH_IMU, NULL);
	if (real_path != NULL) {
		printf("real path returned: %s\n", real_path);
	}
	else {
		printf("failed to read real path: %d\n", errno);
		std::exit(1);
	}

	// Open the serial port associated with the gps device
	int usb_fd = open(real_path, O_RDWR|O_NOCTTY);
	free(real_path);
	if (usb_fd < 0) {
		printf("Unable to open usb port: %d\n", errno);
		std::exit(1);
	}

	// Initialize ROS stuff
	ros::init(argc, argv, avc_common::NODE_NAME_IMU);
	ros::NodeHandle nh;
	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 1000);

	// Initialize the serial connection to the GPS device with correct read settings
	configure_usb(usb_fd);

	char buffer[READ_BUFFER_SIZE];

	while (ros::ok()) {
		int n = read(usb_fd, buffer, sizeof(buffer));

		// sometimes the buffer contains only a newline
		if (n > 1) {
			buffer[n] = 0;
			//printf("buffer string: %s",buffer);

			// Example buffer string contents:
			// buffer string: -0.012193,-0.012547,0.089287,0.000266,0.002264,-0.000799,-0.072839,0.063523,0.052157,0.993951

			std::string word;
			std::stringstream stream(buffer);
			std::vector<std::string> imu_data;
			while( std::getline(stream, word, ',') ) {
				imu_data.push_back(word);
			}

			// order of imu data is thus:
			// lin_acc_x
			// lin_acc_y
			// lin_acc_z
			// ang_vel_x
			// ang_vel_y
			// ang_vel_z
			// orient_x
			// orient_y
			// orient_z
			// orient_w

			sensor_msgs::Imu imu_msg;
			imu_msg.header.stamp = ros::Time::now();
			imu_msg.header.frame_id = imuFrameId;

			imu_msg.linear_acceleration.x = std::atof(imu_data[0].c_str()); //ax;
			imu_msg.linear_acceleration.y = std::atof(imu_data[1].c_str()); //ay;
			imu_msg.linear_acceleration.z = std::atof(imu_data[2].c_str()); //az;
			imu_msg.linear_acceleration_covariance[0] = -1;

			imu_msg.angular_velocity.x = std::atof(imu_data[3].c_str()); //gx;
			imu_msg.angular_velocity.y = std::atof(imu_data[4].c_str()); //gy;
			imu_msg.angular_velocity.z = std::atof(imu_data[5].c_str()); //gz;
			imu_msg.angular_velocity_covariance[0] = -1;

			imu_msg.orientation.x = std::atof(imu_data[6].c_str()); //*(getQ() + 1);
			imu_msg.orientation.y = std::atof(imu_data[7].c_str()); //*(getQ() + 2);
			imu_msg.orientation.z = std::atof(imu_data[8].c_str()); //*(getQ() + 3);
			imu_msg.orientation.w = std::atof(imu_data[9].c_str()); //*(getQ());

			imu_pub.publish(imu_msg);
			ros::spinOnce();
		}
	}
}
