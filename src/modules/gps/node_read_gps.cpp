/*
 * Reads the sparkfun gps module in canonical mode and publishes a gps messagee to ros.
 *
 * utilizes sensors_msgs/NavSatFix.h
 *
 */

// C++ includes
#include <stdio.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdlib>
#include <cstdio>
#include <stdlib.h>

// libnmea includes
#include <nmea.h>
#include <nmea/gpgll.h>
#include <nmea/gpgga.h>
#include <nmea/gprmc.h>

// ROS includes
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

// AVC includes
#include "ros_topics.h"
#include "device_paths.h"
#include "node_names.h"

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

	cfsetispeed(&tty, (speed_t)B9600);
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

	// Initialize ROS stuff
	//ros::init(argc, argv, avc_common::NODE_NAME_GPS);
	//ros::NodeHandle nh;
	//ros::Publisher navsatfix_pub = nh.advertise<sensor_msgs::NavSatFix>(avc_common::ROS_TOPIC_GPS, 10);

	// The GPS device will send messages to this program at a set rate, so that
	// frequency should govern the rate at which ROS messages are sent into the system.
	//ros::Rate loop_rate(10);

	// Initialize the serial connection to the GPS device with correct read settings
	configure_usb(usb_fd);

	char buffer[READ_BUFFER_SIZE];

	while (1) {//ros::ok()) {
		int n = read(usb_fd, buffer, sizeof(buffer));
		if (n > 0) {
			//printf("%.*s",n, buffer);
			buffer[n] = 0;
			printf("buffer string: %s",buffer);

			// Validate the nmea sentence first, "1" tells it to validate the checksum
			if (nmea_validate(buffer, strlen(buffer), 1) == 0) {

				data = nmea_parse(buffer, strlen(buffer), 1);

				if (NULL == data) {
					printf("data is null");
				}
				else {
					sensor_msgs::NavSatFix navsatfix_msg;

					if (NMEA_GPGGA == data->type) {
						nmea_gpgga_s *gpgga = (nmea_gpgga_s *) data;

						printf("GPGGA Sentence\n");
						printf("Number of satellites: %d\n", gpgga->n_satellites);
						printf("Altitude: %d %c\n", gpgga->altitude, gpgga->altitude_unit);
						printf("Longitude:\n");
						printf("  Degrees: %d\n", gpgga->longitude.degrees);
						printf("  Minutes: %f\n", gpgga->longitude.minutes);
						printf("  Cardinal: %c\n", (char) gpgga->longitude.cardinal);
						printf("Latitude:\n");
						printf("  Degrees: %d\n", gpgga->latitude.degrees);
						printf("  Minutes: %f\n", gpgga->latitude.minutes);
						printf("  Cardinal: %c\n", (char) gpgga->latitude.cardinal);
					}

					if (NMEA_GPGLL == data->type) {
						nmea_gpgll_s *gpgll = (nmea_gpgll_s *) data;

						printf("GPGLL Sentence\n");
						printf("Longitude:\n");
						printf("  Degrees: %d\n", gpgll->longitude.degrees);
						printf("  Minutes: %f\n", gpgll->longitude.minutes);
						printf("  Cardinal: %c\n", (char) gpgll->longitude.cardinal);
						printf("Latitude:\n");
						printf("  Degrees: %d\n", gpgll->latitude.degrees);
						printf("  Minutes: %f\n", gpgll->latitude.minutes);
						printf("  Cardinal: %c\n", (char) gpgll->latitude.cardinal);
					}

					if (NMEA_GPRMC == data->type) {
						nmea_gprmc_s *gprmc = (nmea_gprmc_s *) data;
						printf("GPRMC Sentence\n");
						printf("Longitude:\n");
						printf("  Degrees: %d\n", gprmc->longitude.degrees);
						printf("  Minutes: %f\n", gprmc->longitude.minutes);
						printf("  Cardinal: %c\n", (char) gprmc->longitude.cardinal);
						printf("Latitude:\n");
						printf("  Degrees: %d\n", gprmc->latitude.degrees);
						printf("  Minutes: %f\n", gprmc->latitude.minutes);
						printf("  Cardinal: %c\n", (char) gprmc->latitude.cardinal);
					}


					//navsatfix_pub.publish(navsatfix_msg);
					//ros::spinOnce();

					nmea_free(data);
				}
			}
			else {
				printf("nmea sentence NOT valid\n");
			}			
		}
		else {
			printf("read 0 bytes\n");
		}
	}

	// cleanup
	if (close(usb_fd) != 0) {
		printf("Error closing usb port: %d\n", errno);
	}
}
