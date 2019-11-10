#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include "nav_msgs/Odometry.h"

#define DIST_THRESHOLD 0.75f
#define PICKUP_X -3.5
#define PICKUP_Y 4.0
#define DROPOFF_X 3.5
#define DROPOFF_Y 4.0

// Initialize states
bool start = true; // @ start (set goal and display marker @ pickup)
bool pickup = false; // @ picked up point (set goal to dropoff and remove marker @ pickup)
bool dropoff = false; // @ dropped off point (add marker @ drop off)

/**
* Check if robot has reached goal
**/
bool goalReached(nav_msgs::Odometry::ConstPtr msg){
  // Calculates the distance from the robot to the goal
  // and returns true if that distance is less than the threshold

  float Rx = msg->pose.pose.position.x;
  float Ry = msg->pose.pose.position.y;

  float Gx = (start) ? PICKUP_X : DROPOFF_X;
  float Gy = (start) ? PICKUP_Y : DROPOFF_Y;

  float distToGoal = pow( pow(Rx - Gx, 2) + pow(Ry - Gy, 2), 0.5);

  return distToGoal <= DIST_THRESHOLD;
}

/**
* Callback after receiving odom_sub's msg
**/
void callback_odom(const nav_msgs::Odometry::ConstPtr &msg){
  // switches the state when the robot reaches the marker
  if (goalReached(msg)){
      // if start -> start = false, pickup = true;
    if (start && !pickup){
      start = false;
      pickup = true;
      ROS_INFO("Marker picked up");
			ros::Duration(5.0).sleep();
    }
    else if(!start && pickup){
      pickup = false;
      dropoff = true;
      ROS_INFO("Marker dropped off");
			ros::Duration(5.0).sleep();
    }
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  // Publish Marker for rviz
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("marker", 1);
  // Subscribe to the robot's odom
  ros::Subscriber odom_sub = n.subscribe("/odom", 500, callback_odom);

  // Setup marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "add_markers";
  marker.id = 0;
    // Set type (CUBE)
  marker.type = visualization_msgs::Marker::CUBE;
    // Set scale (0.2 x 0.2 x 0.2)
  marker.scale.x = 0.2f;
  marker.scale.y = 0.2f;
  marker.scale.z = 0.2f;
    // Set color (blue)
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;
    // initial position
  marker.pose.position.x = PICKUP_X;
  marker.pose.position.y = PICKUP_Y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

	ROS_INFO("Setting up markers...");
	ros::Duration(5.0).sleep();
	ROS_INFO("Starting...");

  while(ros::ok()){
    marker.lifetime = ros::Duration();

    // @ Start state
    if(start){
      // Display marker @ pick up point
      marker.action = visualization_msgs::Marker::ADD;
      ROS_WARN_ONCE("Marker @ Pick up point");
    }

    //@ Pick up state
    if(pickup){
      // remove marker @ pick up point
      marker.action = visualization_msgs::Marker::DELETE;
      ROS_WARN_ONCE("Marker picked up, deleting marker");
    }

    // @ Drop off state
    if(dropoff){
      // Set and display marker @ drop off point
      marker.pose.position.x = DROPOFF_X;
      marker.pose.position.y = DROPOFF_Y;
      marker.action = visualization_msgs::Marker::ADD;
      ROS_WARN_ONCE("Marker @ Drop off point");
    }

    // @ End state
    if (!dropoff && !pickup && !start){
      ROS_INFO("Marker picked up and dropped off, terminating script...");
      return 0;
    }


    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    marker_pub.publish(marker);
    ros::spinOnce();

  }

  return 0;

}

