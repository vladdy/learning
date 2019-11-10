#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // set up the frame parameters
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();


  // 1) Go to pick up point
  goal.target_pose.pose.position.x = -3.5f;
  goal.target_pose.pose.position.y = 4.0f;
  goal.target_pose.pose.orientation.z = 1.56f;

  ROS_INFO("Moving to pick up point...");
	ros::Duration(5.0).sleep();
  ac.sendGoal(goal);
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Pick up point reached !");
    while(!ac.waitForServer(ros::Duration(5.0))){
       ROS_INFO("Waiting 5s before moving again");
    }
  }
  else
    ROS_INFO("Failed to reach pick up point !");

  // 2) Go to dropoff point
  goal.target_pose.pose.position.x = 3.5f;
  goal.target_pose.pose.position.y = 4.0f;
  goal.target_pose.pose.orientation.z = 1.56f;
  
  ROS_INFO("Moving to drop off point...");
  ac.sendGoal(goal);
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Drop off point reached !");
    while(!ac.waitForServer(ros::Duration(5.0))){
       ROS_INFO("Waiting 5s before moving again");
    }
  }
  else
    ROS_INFO("Failed to reach drop off point !");

  ROS_INFO("Script ended");

  return 0;
}

