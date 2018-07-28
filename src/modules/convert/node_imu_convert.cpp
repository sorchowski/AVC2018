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

#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include "ros_topics.h"
#include "node_names.h"

//ros::NodeHandle nh;

//sensor_msgs::Imu imu_message;
//sensor_msgs::MagneticField  mag_message;

const char magFrameId[] = "mag";
const char imuFrameId[] = "imu";

// Way overkill, but meh...
#define READ_BUFFER_SIZE 4096  

struct termios tty;
struct termios tty_old;

// Will contain parsed gps data
nmea_s * data;

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
	char * real_path = realpath(avc_common::ROS_NODE_DEVICE_PATH_GPS, NULL);
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

	// Initialize the serial connection to the GPS device with correct read settings
	configure_usb(usb_fd);

	char buffer[READ_BUFFER_SIZE];

	while (ros::ok()) {
		int n = read(usb_fd, buffer, sizeof(buffer));
		if (n > 0) {
			//printf("%.*s",n, buffer);
			buffer[n] = 0;
			printf("buffer string: %s",buffer);

//    imu_message.header.stamp = nh.now();
//    mag_message.header.stamp = nh.now();

      // Setup ROS message publisher
//      nh.initNode();
//      nh.advertise(imuPublisher);
//      nh.advertise(magPublisher);

//    imu_message.orientation.x = *(getQ() + 1);
//    imu_message.orientation.y = *(getQ() + 2);
//    imu_message.orientation.z = *(getQ() + 3);
//    imu_message.orientation.w = *(getQ());

//    imu_message.linear_acceleration.x = ax;
//    imu_message.linear_acceleration.y = ay;
//    imu_message.linear_acceleration.z = az;
//    imu_message.linear_acceleration_covariance[0] = -1;

//    imu_message.angular_velocity.x = gx;
//    imu_message.angular_velocity.y = gy;
//    imu_message.angular_velocity.z = gz;
//    imu_message.angular_velocity_covariance[0] = -1;

//      imu_message.header.stamp = nh.now();
//      imu_message.header.frame_id = imuFrameId;
//      mag_message.header.stamp = nh.now();
//      mag_message.header.frame_id = magFrameId;
//      imuPublisher.publish(&imu_message);
//      magPublisher.publish(&mag_message);
//      nh.spinOnce();
        }
    }
}