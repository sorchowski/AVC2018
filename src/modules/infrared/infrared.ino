#include <arduino.h>
#include <ros.h>
#include <sensor_msgs/Range.h>

#include "ros_topics.h"
#include "node_names.h"

#define NUM_SENSORS 3

int analogReadPins[NUM_SENSORS] = {A1, A2, A3};

#define RosPublish false
#define SerialDebug true

ros::NodeHandle nh;

//http://docs.ros.org/api/sensor_msgs/html/msg/Range.html
sensor_msgs::Range range_message;

char ir1FrameId[] = "/ir1";
char ir2FrameId[] = "/ir2";
char ir3FrameId[] = "/ir3";

ros::Publisher infraredPublisher(avc_common::ROS_TOPIC_INFRARED, &range_message);

unsigned long timer = 0;

void setFrameId(int sensorNum) {
  switch(sensorNum) {
    case 1:
      range_message.header.frame_id = ir1FrameId;
      break;
    case 2:
      range_message.header.frame_id = ir2FrameId;
      break;
    case 3:
      range_message.header.frame_id = ir3FrameId;
      break;
    default:
      range_message.header.frame_id = "irUnknown";
      break;
  }
}

void setup()
{
  if (SerialDebug) {
    Serial.begin(115200);
  }

  if (RosPublish) {
    range_message.radiation_type = sensor_msgs::Range::INFRARED;
    range_message.field_of_view = 0.0; // rads, set to zero due to no value given in datasheet
    range_message.min_range = 1.0; //meters
    range_message.max_range = 5.5; //meters

    // Setup ROS message publisher
    nh.initNode();
    nh.advertise(infraredPublisher);
  }
}

void loop()
{
  if ((millis()-timer) > 100) { // 10 times a second, hopefully
    timer = millis();

    for (int i=1;i<=NUM_SENSORS;i++) {
      int infraredReadValue = analogRead(analogReadPins[i-1]);

      // TODO: convert analog value to distance measurement
      float calculatedDistance = infraredReadValue;

      if (SerialDebug) {
        String calculatedDistanceValueStr = String(calculatedDistance);
        Serial.println(calculatedDistanceValueStr);
      }

      if (RosPublish) {
        range_message.header.stamp = nh.now();
        range_message.range = calculatedDistance;
        setFrameId(i);
        infraredPublisher.publish(&range_message);
        nh.spinOnce();
      }
    }
  }
}
