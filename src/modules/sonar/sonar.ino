#include <arduino.h>
#include <SonarArray.h>

#include <ros.h>
#include <sensor_msgs/Range.h>

#include "ros_topics.h"
#include "node_names.h"

#define NUM_SENSORS 7

#define RosPublish true
#define SerialDebug false

// Every 60 milliseconds per HCSR04 documented recommendation
#define SAMPLE_RATE 60

#define MAX_RANGE 4.0
#define MIN_RANGE 0.02

ros::NodeHandle nh;

//http://docs.ros.org/api/sensor_msgs/html/msg/Range.html
sensor_msgs::Range range_message;

ros::Publisher sonarPublisher(avc_common::ROS_TOPIC_RANGE, &range_message);

Sonar::SonarArray sonarArray(NUM_SENSORS);

unsigned long timer = 0;

// Sensor numbers are 1-based
unsigned int current_sensor_num = 1;

void setFrameId(int sensorNum) {
  switch(sensorNum) {
    case 1:
      range_message.header.frame_id = "s1";
      break;
    case 2:
      range_message.header.frame_id = "s2";
      break;
    case 3:
      range_message.header.frame_id = "s3";
      break;
    case 4:
      range_message.header.frame_id = "s4";
      break;
    case 5:
      range_message.header.frame_id = "s5";
      break;
    case 6:
      range_message.header.frame_id = "s6";
      break;
    case 7:
      range_message.header.frame_id = "s7";
      break;
    case 8:
    default:
      break;
  }
}

void setup() {
  if (SerialDebug) {
    Serial.begin(115200);
  }

  if (RosPublish) {
    range_message.radiation_type = sensor_msgs::Range::ULTRASOUND;
    // https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
    range_message.field_of_view = 0.261799; // (in rads) = 15degrees
    range_message.min_range = MIN_RANGE; // (in meters)
    range_message.max_range = MAX_RANGE; // (in meters)
    // Setup ROS message publisher
    nh.initNode();
    nh.advertise(sonarPublisher);
  }
}

void loop() {

  if ((millis() - timer) > SAMPLE_RATE) {
    timer = millis();

    sonarArray.scan(current_sensor_num);

    float distance = sonarArray.getDistance(current_sensor_num);
    // The distance value is in cm. We need to convert it to meters per ROS conventions.

    float calculatedDistance = distance/100.0;

    if (SerialDebug) {
      String distanceStr = String(current_sensor_num)+": "+String(calculatedDistance);
      Serial.println(distanceStr);
    }

    if (RosPublish) {
      // Do not publish a range message if the calculated distance is outside of the max/min of the sonar sensor.
      if (calculatedDistance<MAX_RANGE || calculatedDistance>MIN_RANGE) {
        range_message.header.stamp = nh.now();
        range_message.range = calculatedDistance;
        setFrameId(current_sensor_num);
        sonarPublisher.publish(&range_message);
        nh.spinOnce();
      }
    }

    current_sensor_num++;

    if (current_sensor_num > NUM_SENSORS) {
      current_sensor_num = 1;
    }
  }
}
