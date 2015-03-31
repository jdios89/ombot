#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <cmath>
#include "std_msgs/Bool.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_broadcaster.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
double x ;
double y ;
bool enabled;
double th = 0.0;
bool activated = false;
bool last_bool = false;
void start_move(const std_msgs::Bool::ConstPtr& activation)
{
  activated = activation->data;
} 

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  ros::Subscriber act = n.subscribe("get_close", 10, start_move);
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  ros::Rate r(10.0);
  

  while (ros::ok() && !activated)
  {
    ros::spinOnce();
    r.sleep();  
  }
  ros::param::param<bool>("/move_base/global_costmap/simple_layer_sub/enabled", enabled, false);
    if( enabled != last_bool)
    {
         last_bool = enabled;
         if(enabled) ROS_INFO("Got object location!");
         else ROS_INFO("No object");
         
    }
    
    //ROS_INFO("Enabled ");
    else {}
    //ROS_INFO("Not enabled");
    ros::param::param<double>("/move_base/global_costmap/simple_layer_sub/object_x", x, 0.0);
    ROS_INFO(" GOT x %f", x);
    ros::param::param<double>("/move_base/global_costmap/simple_layer_sub/object_y", y, 0.0);
    ROS_INFO(" GOT y %f", y);
  move_base_msgs::MoveBaseGoal goal;
  
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";  //The frame of the goal
  goal.target_pose.header.stamp = ros::Time::now();
  
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(th);
  goal.target_pose.pose.orientation = quat;

  ROS_INFO("Sending behind object");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Move behind object");
  else
    ROS_INFO("Could not get behind object");

  return 0;
}
