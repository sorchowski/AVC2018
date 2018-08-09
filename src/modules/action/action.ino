#include <arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

#define RosListen true
#define SerialDebug false

ros::NodeHandle nh;

void handleCmd(const geometry_msgs::Twist velocity_cmd) {
    float x_vel = velocity_cmd.linear.x;
    float y_vel = velocity_cmd.linear.y;
    float theta_vel = velocity_cmd.angular.z;

    // TODO: map the steering (theta_vel) to a servo value and output

    // TODO: map the velocities to a esc setting and output
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &handleCmd);

void setup()
{
  if (SerialDebug) {
    Serial.begin(115200);
  }

  if (RosListen) {
    // Setup ROS message publisher
    nh.initNode();
    nh.subscribe(sub);
  }
}

void loop()
{
    nh.spinOnce();
    delay(1);
}
