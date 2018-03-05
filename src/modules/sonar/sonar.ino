#include <arduino.h>
#include <SonarArray.h>

#include <ros.h>
#include <sensor_msgs/Range.h>

#include "ros_topics.h"
#include "node_names.h"

#define NUM_SENSORS 7

#define RosPublish false
#define SerialDebug true

ros::NodeHandle nh;

//http://docs.ros.org/api/sensor_msgs/html/msg/Range.html
sensor_msgs::Range range_message;

ros::Publisher sonarPublisher(avc_common::ROS_TOPIC_SONAR, &range_message);

Sonar::SonarArray sonarArray(NUM_SENSORS);

unsigned long timer = 0;

void setFrameId(int sensorNum) {
  switch(sensorNum) {
    case 1:
      range_message.header.frame_id = "/sonar1";
      break;
    case 2:
      range_message.header.frame_id = "/sonar2";
      break;
    case 3:
      range_message.header.frame_id = "/sonar3";
      break;
    case 4:
      range_message.header.frame_id = "/sonar4";
      break;
    case 5:
      range_message.header.frame_id = "/sonar5";
      break;
    case 6:
      range_message.header.frame_id = "/sonar6";
      break;
    case 7:
      range_message.header.frame_id = "/sonar7";
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
    range_message.min_range = 4.0; // (in meters)
    range_message.max_range = 0.02; // (in meters)
    // Setup ROS message publisher
    nh.initNode();
    nh.advertise(sonarPublisher);
  }
}

void loop() {

  if ((millis() - timer) > 500) { // twice a second, can we do better?
    timer = millis();

    for (int i=1;i<=NUM_SENSORS;i++) {
      sonarArray.scan(i);

      float distance = sonarArray.getDistance(i);
      // TODO: convert the distance value?
      float calculatedDistance = distance;

      if (SerialDebug) {
        String distanceStr = "D: "+String(calculatedDistance);  
        Serial.println(distanceStr);
      }

      if (RosPublish) {
        range_message.header.stamp = nh.now();

        range_message.range = calculatedDistance;
        setFrameId(i);
        sonarPublisher.publish(&range_message);
        nh.spinOnce();
      }

      // TODO: determine a good value for below
      delay(50);
    }
  }
}
