#include <ros/ros.h>
#include <ros/console.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
move_base_msgs::MoveBaseGoal create_goal(double lin_x, double lin_y, double ang_z, std::string ref_frame);
bool reach_goal(MoveBaseClient &ac_client, move_base_msgs::MoveBaseGoal goal);

int main(int argc, char** argv){
  ros::init(argc, argv, "send_simple_goal");
  ros::NodeHandle nh("~");
  double goal_pos_x, goal_pos_y, goal_rot_z;
  std::string ref_frame_id;
  nh.getParam("goal_pos_x", goal_pos_x);
  nh.getParam("goal_pos_y", goal_pos_y);
  nh.getParam("goal_rot_z", goal_rot_z);
  nh.getParam("ref_frame_id", ref_frame_id);
  goal_rot_z = goal_rot_z * 3.1415 / 180.0;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // desired goal location and orientation
  move_base_msgs::MoveBaseGoal my_goal;
  my_goal = create_goal(goal_pos_x, goal_pos_y, goal_rot_z, ref_frame_id);
  ROS_INFO("Sending goal in %s with: [x, y, rot_z] = [%f, %f, %f]", ref_frame_id.c_str(), goal_pos_x, goal_pos_y, goal_rot_z);
  bool goal_status = reach_goal(ac, my_goal);
  ROS_INFO_COND(goal_status, "Goal reached");
}

// create goal object according to given position and orientaion
move_base_msgs::MoveBaseGoal create_goal(double lin_x, double lin_y,double ang_z, std::string ref_frame){
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = ref_frame;
  goal.target_pose.header.stamp = ros::Time::now();

  // add linear movement to goal
  goal.target_pose.pose.position.x = lin_x;
  goal.target_pose.pose.position.y = lin_y;

  // add angular movement to goal
  tf2::Quaternion q_rot;
  double roll=0, pitch=0, yaw=ang_z;
  q_rot.setRPY(roll, pitch, yaw);
  q_rot.normalize();
  tf2::convert(q_rot, goal.target_pose.pose.orientation);

  return goal;
}

// send the goal to robot and see what happens
bool reach_goal(MoveBaseClient &ac_client, move_base_msgs::MoveBaseGoal goal){
  ac_client.sendGoal(goal);
  ac_client.waitForResult();
  bool goal_reached = ac_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
  return goal_reached;
}
