#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending Pick Up goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the robot has reached the pick up zone");
  else
    ROS_INFO("ERROR: the robot failed to reach Pick Up zone");

  //DONE: add pause 5 seconds
  ros::Duration(5.0).sleep();  // Sleep for 5 seconds

  // 2. Add Drop off goal
  // 2.1 DONE: add new goal
  goal.target_pose.pose.position.x = 2.0;
  goal.target_pose.pose.orientation.w = 1.0;
   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending Drop Off goal");
  ac.sendGoal(goal);

  //2.2 wait for results
  ac.waitForResult();

  //2.3 check if robot reached its goal 
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, robot has reachend Drop Off zone");
  else
    ROS_INFO("ERROR: the robot failed to reach Drop Off zone!");

  return 0;
}
