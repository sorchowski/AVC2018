// Taken from http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals

// http://wiringpi.com/
#include <wiringPi.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <fstream>
#include <string>


#define PIN_LED_RED 14
#define PIN_LED_GREEN 13
#define PIN_SWITCH 12
#define WAYPOINTS_FILE_PARAMETER "waypoints"
#define PACKAGE "avc"


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void flashLED(int ledPin, int numSecondsBeforeExit) {
  for (int i=0;i<numSecondsBeforeExit; i++) {
    digitalWrite(PIN_LED_RED, HIGH);
    ros::Duration(0.5).sleep();
    digitalWrite(PIN_LED_RED, LOW);
    ros::Duration(0.5).sleep();
  }
}

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

  ros::init(argc, argv, "waypoint_follower");

  ros::NodeHandle nh;

  // Get the filename of the waypoints
  std::string waypointsFile;
  if (!nh.hasParam(WAYPOINTS_FILE_PARAMETER)) {
    ROS_ERROR("This node requires the waypoints file parameter");
    flashLED(PIN_LED_RED, 15);
    return 0;
  }
  nh.getParam(WAYPOINTS_FILE_PARAMETER, waypointsFile);
  //std::string packagePath = ros::package::getPath(PACKAGE);
  ROS_INFO("SEO: got waypoints file parameter: %s", waypointsFile);

  // Read the waypoints file
  std::vector<string> waypoints;
  std::ifstream waypointsfile(waypointsFile);
  std::string line;
  while( std::getline(waypointsfile, line)) {
    std::istringstream waypoint(line);
    //int a, b;
    //if (!(iss >> a >> b)) { break; } // error
    waypoints.add(line);
  }

  ROS_INFO("Read %d waypoints",waypoints.length);


  return 0;



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

  int switchResult = 1;
  while (switchResult) {
    switchResult = digitalRead(PIN_SWITCH);
    if (switchResult==0) {
      // Go to sleep for a bit to make sure it wasn't spurious 
      ros::Duration(0.1).sleep();
      switchResult = digitalRead(PIN_SWITCH);
    }
    else {
      ros::Duration(0.1).sleep();
    }
  }
  ROS_INFO("Received GO signal");


  // Read file in
  // For each element:
  //   If it's a waypoint set a goal, send it, wait for response
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "odom";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 4.0;
    goal.target_pose.pose.position.y = 1.0;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Completed Goal, moving to next item.");
    } else {
      ROS_INFO("Failed to complete goal, stopping.");
    }

  //   If it's a pose, set it, send it




  return 0;
}
