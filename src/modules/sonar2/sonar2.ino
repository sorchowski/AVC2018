#include <arduino.h>

#include <ros.h>
#include <sensor_msgs/Range.h>

#include "ros_topics.h"
#include "node_names.h"

#define NUM_SENSORS 7
#define TIMEOUT 60000
#define RosPublish true
#define SerialDebug false

// Every 60 milliseconds per HCSR04 documented recommendation
#define SAMPLE_RATE 60

#define MAX_RANGE 4.0
#define MIN_RANGE 0.02

#define PIN_TRIGGER_1 9
#define PIN_TRIGGER_2 8
#define PIN_TRIGGER_3 7
#define PIN_TRIGGER_4 6
#define PIN_TRIGGER_5 5
#define PIN_TRIGGER_6 4
#define PIN_TRIGGER_7 3
#define PIN_TRIGGER_8 2

#define PIN_ECHO1 10
#define PIN_ECHO2 11

ros::NodeHandle nh;

//http://docs.ros.org/api/sensor_msgs/html/msg/Range.html
sensor_msgs::Range range_message;

ros::Publisher sonarPublisher(avc_common::ROS_TOPIC_RANGE, &range_message);

unsigned long timer = 0;

const int triggerAssignments[NUM_SENSORS] = {PIN_TRIGGER_1, PIN_TRIGGER_2, PIN_TRIGGER_3, PIN_TRIGGER_4, PIN_TRIGGER_5, PIN_TRIGGER_6, PIN_TRIGGER_7};
const int echoAssignments[NUM_SENSORS] = {PIN_ECHO1, PIN_ECHO1, PIN_ECHO1, PIN_ECHO1, PIN_ECHO2, PIN_ECHO2, PIN_ECHO2};

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

  for (int i=0;i<NUM_SENSORS;i++) {
    int triggerPin = triggerAssignments[i];
    int echoPin = echoAssignments[i]; 
    pinMode(triggerPin, OUTPUT);
    digitalWrite(triggerPin, LOW);
    pinMode(echoPin, INPUT);
  }
}

void loop() {

  if ((millis() - timer) > SAMPLE_RATE) {
    timer = millis();

    int currentTriggerPin = triggerAssignments[current_sensor_num-1];
    int currentEchoPin = echoAssignments[current_sensor_num-1];
    float distance = scan(currentTriggerPin, currentEchoPin);

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

long scan(int triggerPin, int echoPin) {
  long duration;

  // Pulse the trigger pin
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);  // per spec, pulse should be 10us
  digitalWrite(triggerPin, LOW);

  duration = pulseIn(echoPin, HIGH, TIMEOUT);
  // duration is in microseconds

  // 1/29.1 = 0.0343 c/uS. Also divide by two to account for the sound wave travelling out AND back.
  return (duration/2)/29.1;
}
