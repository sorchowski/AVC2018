#include <LS7366R.h>
#include <SPI.h>

#include <ros.h>
#include <geometry_msgs/TwistStamped.h>

#include "ros_topics.h"
#include "node_names.h"

#define RosPublish true
#define SerialDebug false

#define ENCODER1_SS 10
#define ENCODER2_SS 11

ros::NodeHandle nh;
geometry_msgs::TwistStamped odom_message;
char odometerFrameId[] = "/odometer";

ros::Publisher odoPub(avc_common::ROS_TOPIC_ODOMETRY, &odom_message);

using namespace LS7366R;

byte MDR0_CONFIG = MDR0_X1QUAD|MDR0_FREE_CNT|MDR0_IDX_DISABLE|MDR0_FILTR_CLK_DIV1;
byte MDR1_CONFIG = MDR1_CNT_4BYTE|MDR1_FLAG_NONE;

QuadratureEncoder encoder1(ENCODER1_SS, MDR0_CONFIG, MDR1_CONFIG);
QuadratureEncoder encoder2(ENCODER2_SS, MDR0_CONFIG, MDR1_CONFIG);

unsigned long timer = 0;

void setup(){
  if (SerialDebug) {
    Serial.begin(115200);
  }

  pinMode(ENCODER1_SS, OUTPUT);
  pinMode(ENCODER2_SS, OUTPUT);
  SPI.begin();
  encoder1.init();
  //encoder2.init();

  if (RosPublish) {
    // Setup ROS message publisher
    nh.initNode();
    nh.advertise(odoPub);
  }
}

void loop() {
  if ((millis() - timer) > 100) { // 10 times a second, hopefully
    timer = millis();

    long count1 = encoder1.count();
    byte status1 = encoder1.status();

    //long count2 = encoder2.count();
    //byte status2 = encoder2.status();

    // TODO: convert "count" to distance measurement
    // Do some math to convert tick count to revolutions of wheel
    // Average the wheel rotations between right and left.

    float velocity = 1.234; // m/s

    if (SerialDebug) {
      String debugMsgStr = "Count: "+String(count1)+", status: "+String(status1);
      Serial.println(debugMsgStr);
    }

    if (RosPublish) {
      odom_message.header.stamp = nh.now();
      odom_message.header.frame_id = odometerFrameId;
      odom_message.twist.linear.x = velocity; // TODO: decide whether this should be x,y,or z
      odoPub.publish(&odom_message);
      nh.spinOnce();
    }
  }
}
