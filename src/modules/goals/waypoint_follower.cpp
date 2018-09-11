// Taken from http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals

// http://wiringpi.com/
#include <wiringPi.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseResult.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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

class AvcGoals
{

  private:
    MoveBaseClient client;
    std::vector<std::string> waypoints;
    std::string waypointsFile;
    std::vector<int>::size_type currentGoal;
    ros::Publisher set_pose_pub;
    ros::NodeHandle nh;

    bool sendGoal(std::vector<int>::size_type goalNum) {

      std::istringstream waypoint(waypoints[goalNum]);
      currentGoal++;
      std::string command;
      float x, y, ox,oy,oz,ow;
      waypoint >> command >> x >> y >> ox >> oy >> oz >> ow;
      if (command == "goal") {
        ROS_INFO("SEO: Sending goal");
        // If it's a waypoint, set a goal, send it, wait for response

        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "odom";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.orientation.x = ox;
        goal.target_pose.pose.orientation.y = oy;
        goal.target_pose.pose.orientation.z = oz;
        goal.target_pose.pose.orientation.w = ow;

        client.sendGoal(goal, boost::bind(&AvcGoals::avcGoalCb, this, _1, _2));

        return false;
      }
      else if (command == "pose") {
        // If it's a pose, set it, send it
        ROS_INFO("SEO: Sending pose");
        geometry_msgs::PoseWithCovarianceStamped pose;

        pose.header.frame_id = "odom";
        pose.header.stamp = ros::Time::now();

        pose.pose.pose.position.x = x;
        pose.pose.pose.position.y = y;
        pose.pose.pose.orientation.x = ox;
        pose.pose.pose.orientation.y = oy;
        pose.pose.pose.orientation.z = oz;
        pose.pose.pose.orientation.w = ow;
        // 0             7             14            21            28            35
       //[1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1]
        pose.pose.covariance[0] = 1;
        pose.pose.covariance[7] = 1;
        pose.pose.covariance[14] = 1;
        pose.pose.covariance[21] = 1;
        pose.pose.covariance[28] = 1;
        pose.pose.covariance[35] = 1;
        set_pose_pub.publish(pose);
        ros::Duration(1.0).sleep();
        return true;
     }
   }

  public:
    AvcGoals(std::string waypointsFile) :
        client("move_base", true),
        waypointsFile(waypointsFile),
        currentGoal(0) {

      std::ifstream waypointsfile(waypointsFile.c_str());
      std::string line;
      while( std::getline(waypointsfile, line)) {
        std::istringstream waypoint(line);
        std::string command;
        float x, y, ox,oy,oz,ow;
        if (!(waypoint >> command >> x >> y >> ox >> oy >> oz)) {
          ROS_ERROR("waypoints file is not in the correct format");
          break;
        }
        std::string data = line;
        waypoints.push_back(data);
      }
      ROS_INFO("SEO: loaded %d waypoints", waypoints.size());
      set_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("set_pose", 1000);
      
      //wait for the action server to come up
      while(!client.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
      }
      ROS_INFO("move_base server is now up");

    }
 
    void start() {
      ROS_INFO("SEO: hit start");
      if (currentGoal != waypoints.size()) {
        bool wasPose = sendGoal(currentGoal);
        if (wasPose) {
          ROS_INFO("SEO: was pose");
          sendGoal(currentGoal);
        }
        else {
          ROS_INFO("SEO: was not pose");
        }
      } else {
        ROS_INFO("SEO: in start, shutting down");
        ros::shutdown();
      }
    }

    void avcGoalCb(const actionlib::SimpleClientGoalState& state, 
                   const move_base_msgs::MoveBaseResult::ConstPtr&
                  ) {
      ROS_INFO("SEO: got goal callback");

      if(state  == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Completed Goal, moving to next item.");
      } else {
        ROS_INFO("Failed to complete goal, stopping");
      }
      if (currentGoal != waypoints.size()) {
        bool wasPose = sendGoal(currentGoal);
        if (wasPose) {
          ROS_INFO("SEO: in callback, was pose");
          sendGoal(currentGoal);
        }
        else {
          ROS_INFO("SEO: was not pose");
        }
      } else {
        ROS_INFO("SEO: in callback, shutting down");
        ros::shutdown();
      }
    }

};


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

  //ros::Publisher set_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("set_pose", 1000);

  std::string param = ros::this_node::getName()+"/"+WAYPOINTS_FILE_PARAMETER;

  // Get the filename of the waypoints
  std::string waypointsFile;
  if (!nh.hasParam(param.c_str())) {
    ROS_ERROR("This node requires the waypoints file parameter");
    flashLED(PIN_LED_RED, 15);
    return 0;
  }
  nh.getParam(param.c_str(), waypointsFile);
  ROS_INFO("Got waypoints file parameter: %s", waypointsFile.c_str());

  AvcGoals avcGoals(waypointsFile);

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

  avcGoals.start();

  ros::spin();

  ROS_INFO("SEO: completed following waypoints");

  return 0;
}

// Example set pose: rostopic pub /set_pose --once geometry_msgs/PoseWithCovarianceStamped '{header: {stamp: now, frame_id: "odom"}, pose: {pose: {position: {x: 4.0, y: 1.0, z: 0.0}, orientation: {w: 1.0} }, covariance: [1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1]}}'

