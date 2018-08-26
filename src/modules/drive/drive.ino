#include <arduino.h>
#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

#define RosListen true
#define SerialDebug false

//0.0 to 2.0 for 1500 to 2000, forward
//-2.0 to 0.0 for 1000 to 1500, reverse
//-2.0m/s to 2.0 for 1000us to 2000
// 0-4 to 0-1000

#define MAX_VELOCITY 2.0

#define ESC_VELOCITY_MAX_REVERSE 1000
#define ESC_VELOCITY_ZERO 1500
#define ESC_VELOCITY_MAX_FORWARD 2000

#define SERVO_MAX_LEFT 1000
#define SERVO_CENTER 1500
#define SERVO_MAX_RIGHT 2000

#define STEERING_MAX_THETA M_PI/2

ros::NodeHandle nh;

float current_x_vel = 0.0;
float current_y_vel = 0.0;
float current_theta_vel = 0.0;

Servo steeringServo;
Servo escServo;

int mapThetaToServo(float theta) {
  // TEB steering for non-holonomic (car-like) vehicles ranges from
  // -PI/2 to PI/2

  // TODO: is negative left or right? positive seems like it should be left if 0 is straight ahead.
  int steeringResult = SERVO_CENTER;
  float steeringRatio = theta/STEERING_MAX_THETA;

  if (steeringRatio < 0.0) {
    // Map theta to between SERVO_CENTER and SERVO_MAX_RIGHT
    steeringRatio = steeringRatio*(-1.0);
    steeringResult = int(((SERVO_MAX_RIGHT-SERVO_CENTER)*steeringRatio)+SERVO_CENTER);
  } else if (theta > 0.0) {
    // Map theta to between SERVO_CENTER and SERVO_MAX_LEFT
    steeringResult = int(((SERVO_CENTER-SERVO_MAX_LEFT)*steeringRatio)+SERVO_MAX_LEFT);
  }

  return steeringResult;
}

int mapVelocityToServo(float input_velocity) {
  // add max_velocity to input
  // upper velocity range = max_velocity*2
  float upper_velocity_range = MAX_VELOCITY*2.0;

  // temp should be between 0.0 and 2*MAX_VELOCITY
  float temp_velocity_value = input_velocity+MAX_VELOCITY; // We always assume max forward = abs(max reverse)

  float target_range = ESC_VELOCITY_MAX_FORWARD-ESC_VELOCITY_MAX_REVERSE;

  // now map temp velocity to target range
  float temp = (temp_velocity_value/upper_velocity_range)*target_range;

  // adjust the range back to our expected esc values
  int final_esc_velocity = temp+ESC_VELOCITY_MAX_REVERSE;

  return final_esc_velocity;
}

void handleCmd(const geometry_msgs::Twist velocity_cmd) {
    float x_vel = velocity_cmd.linear.x;
    float y_vel = velocity_cmd.linear.y;
    float theta_vel = velocity_cmd.angular.z;

    // TEB Local Planner uses x velocity for forward/backward and angular Z for steering.

    int steeringCmd = mapThetaToServo(theta_vel);
    steeringServo.writeMicroseconds(steeringCmd);

    int velocityCmd = mapVelocityToServo(x_vel);
    // If current velocity is + and new velocity is -, we need to reverse. The Traxxas
    // esc requires the period to be set to 1500us for a brief amount of time before
    // setting the velocity to reverse. i.e. we can't go, for example, from 1700us
    // directly to 1300us.
    if (current_x_vel > 0.0 && x_vel < 0.0) {
      // Stop the vehicle, set speed to 0 (i.e. 1500us)
      escServo.writeMicroseconds(ESC_VELOCITY_ZERO);
      delay(100);
    }

    escServo.writeMicroseconds(velocityCmd);

    current_x_vel = x_vel;
    current_y_vel = y_vel;
    current_theta_vel = theta_vel;
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

  // Zero everything out before we start processing ROS commands
  escServo.writeMicroseconds(ESC_VELOCITY_ZERO);
  steeringServo.writeMicroseconds(SERVO_CENTER);
}

void loop()
{
    nh.spinOnce();
    delay(5);
}
