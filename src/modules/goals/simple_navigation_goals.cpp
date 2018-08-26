// Taken from http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals

// http://wiringpi.com/
#include <wiringPi.h>

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#define PIN_LED_RED 10
#define PIN_LED_GREEN 9
#define PIN_SWITCH 11

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){

  // Setup wiringPi and pin modes
  wiringPiSetup();
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_SWITCH, INPUT);

  // Turn the NOT ready indicator on (this should be a red LED).
  digitalWrite(PIN_LED_RED, HIGH);
  // Turn the ready indicator off (this should be the green LED).
  digitalWrite(PIN_LED_GREEN, LOW);

  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  ROS_INFO("move_base server is now up");

  // Turn the NOT ready indicator off (this should be a red LED).
  digitalWrite(PIN_LED_RED, LOW);
  // Turn the ready indicator on (this should be the green LED).
  digitalWrite(PIN_LED_GREEN, HIGH);

  // TODO: Wait for input from the go switch.
  int sitchResult = 1;
  while (switchResult) {
    int switchResult = digitalRead(PIN_SWITCH);
    if (switchResult==false) {
      // Go to sleep for a bit to make sure it wasn't spurious 
      ros::Duration(0.1).sleep();
      switchResult = digitalRead(PIN_SWITCH);
    }
    else {
      ros::Duration(0.1).sleep();
    }
  }

  return 0;
  /*
  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
  */
}
