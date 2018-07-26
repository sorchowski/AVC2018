#include <LS7366R.h>
#include <SPI.h>

#include <ros.h>
#include <geometry_msgs/TwistStamped.h>

#include "ros_topics.h"
#include "node_names.h"

#define RosPublish true
#define SerialDebug false

#define ENCODER1_SS 5
#define ENCODER2_SS 11

// 10 times a second
#define SAMPLE_RATE 100

// 4.25 inches in meters
#define WHEEL_DIAMETER 0.10795

ros::NodeHandle nh;
geometry_msgs::TwistStamped odom_message;

ros::Publisher odoPub(avc_common::ROS_TOPIC_ODOMETRY, &odom_message);

using namespace LS7366R;

byte MDR0_CONFIG = MDR0_X1QUAD|MDR0_FREE_CNT|MDR0_IDX_DISABLE|MDR0_FILTR_CLK_DIV1;
byte MDR1_CONFIG = MDR1_CNT_4BYTE|MDR1_FLAG_NONE;

QuadratureEncoder encoder1(ENCODER1_SS, MDR0_CONFIG, MDR1_CONFIG);
QuadratureEncoder encoder2(ENCODER2_SS, MDR0_CONFIG, MDR1_CONFIG);

unsigned long timer = 0;
unsigned long last_count1 = 0;
unsigned long last_count2 = 0;

void setup(){
  if (SerialDebug) {
    Serial.begin(115200);
  }

  pinMode(ENCODER1_SS, OUTPUT);
  pinMode(ENCODER2_SS, OUTPUT);
  SPI.begin();
  encoder1.init();
  encoder2.init();

  if (RosPublish) {
    // Setup ROS message publisher
    nh.initNode();
    nh.advertise(odoPub);
  }
}

void loop() {
  if ((millis() - timer) > SAMPLE_RATE) {
    timer = millis();

    unsigned long count1 = encoder1.count();
    byte status1 = encoder1.status();

    unsigned long count2 = encoder2.count();
    byte status2 = encoder2.status();

    signed long signed_diff1 = count1-last_count1;
    unsigned long ticks1 = abs(signed_diff1);
    signed long signed_diff2 = count2-last_count2;
    unsigned long ticks2 = abs(signed_diff2);

    bool forward = (signed_diff1 < 0) ? false : true;
    // Consider obtaining direction from status byte. Note, testing revealed this
    // did not always seem to be reliable.
    float velocity1 = (((float)ticks1)*(M_PI*WHEEL_DIAMETER))/20.0;
    float velocity2 = (((float)ticks2)*(M_PI*WHEEL_DIAMETER))/20.0;

    // Convert from velocity/100ms to velocity/s
    velocity1 = velocity1*10;
    velocity2 = velocity2*10;

    // Average the wheel rotations between right and left.
    float velocity = (velocity1+velocity2)/2; // in m/s

    last_count1=count1;
    last_count2=count2;

    if (SerialDebug) {
      String debugMsgStr = "Count: "+String(count1)+", status: "+String(status1);
      Serial.println(debugMsgStr);
      String debugString2 = "V: "+String(velocity);
      Serial.println(debugString2);
    }

    if (RosPublish) {
      odom_message.header.stamp = nh.now();
      odom_message.header.frame_id = "odom";
      //odom_message.child_frame_id = "base_link";
      odom_message.twist.linear.x = velocity;
      odoPub.publish(&odom_message);
      nh.spinOnce();
    }
  }
}
