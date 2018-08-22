#include <arduino.h>
#include <ros.h>
#include <sensor_msgs/Range.h>

#include "ros_topics.h"
#include "node_names.h"

#define NUM_SENSORS 3

int analogReadPins[NUM_SENSORS] = {A1, A2, A3};

#define RosPublish true
#define SerialDebug false

// 20 times per second
#define SAMPLE_RATE 50

#define MAX_RANGE 5.0
#define MIN_RANGE 1.0

ros::NodeHandle nh;

//http://docs.ros.org/api/sensor_msgs/html/msg/Range.html
sensor_msgs::Range range_message;

const char ir1FrameId[] = "ir_left";
const char ir2FrameId[] = "ir_center";
const char ir3FrameId[] = "ir_right";

ros::Publisher infraredPublisher(avc_common::ROS_TOPIC_RANGE, &range_message);

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
    range_message.min_range = MIN_RANGE; //meters
    range_message.max_range = MAX_RANGE; //meters

    // Setup ROS message publisher
    nh.initNode();
    nh.advertise(infraredPublisher);
  }
}

void loop()
{
  if ((millis()-timer) > SAMPLE_RATE) {
    timer = millis();

    for (int i=1;i<=NUM_SENSORS;i++) {
      int infraredReadValue = analogRead(analogReadPins[i-1]);

      // 2.5v, 100cm, 512
      // 2.0v, 150cm, 409
      // 1.75v, 200cm, 358
      // 1.6v, 300cm, 327
      // 1.45v, 400cm, 296
      // ~1.36v at 500cm, 280
      // note this is not linear. This is a best effort.

      int calculatedDistance = 0;

      if (infraredReadValue < 513 || infraredReadValue > 279) {
        // values lower than 1.0m and higher than 5.0m are outside the
        // possible range of the infrared sensors. These values will be
        // used to clear existing point values.

        if (infraredReadValue <= 512 && infraredReadValue >= 409) {
          calculatedDistance = map(infraredReadValue, 409, 512, 150, 100);
        } else if (infraredReadValue < 409 && infraredReadValue >= 358) {
          calculatedDistance = map(infraredReadValue, 358, 409, 200, 150);
        } else if (infraredReadValue < 358 && infraredReadValue >= 327) {
          calculatedDistance = map(infraredReadValue, 327, 358, 300, 200);
        } else if (infraredReadValue < 327 && infraredReadValue >= 296) {
          calculatedDistance = map(infraredReadValue, 296, 327, 400, 300);
        } else {
          calculatedDistance = map(infraredReadValue, 280, 296, 500, 400);
        }
      }

      // convert from centimeters
      float range = calculatedDistance;
      range = range/100.0;

      if (SerialDebug) {
        String calculatedDistanceValueStr = String(i)+": "+String(calculatedDistance);
        Serial.println(calculatedDistanceValueStr);
      }

      if (RosPublish) {
        range_message.header.stamp = nh.now();
        range_message.range = range;
        setFrameId(i);
        infraredPublisher.publish(&range_message);
        nh.spinOnce();
      }
    }
  }
}
