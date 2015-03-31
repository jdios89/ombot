#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <cmath>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>
#include <string>
#include <geometry_msgs/Quaternion.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Empty.h>

dynamic_reconfigure::ReconfigureRequest srv_req;
dynamic_reconfigure::ReconfigureResponse srv_resp;
dynamic_reconfigure::DoubleParameter double_param;
dynamic_reconfigure::BoolParameter bool_param;
//dynamic_reconfigure::Config conf;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
double x ;
double y ;
bool enabled;
double th = 0.0;
bool activated = false;
bool last_bool = false;
double x_goal = 0.0;
double y_goal = 0.0;
double yaw = 0.0;
geometry_msgs::Quaternion goal_quat;

void start_move(const std_msgs::Bool::ConstPtr& activation) //For debugging, start the task by sending a bool
{
  activated = activation->data;
} 

void write_to_costmap(double position_x, double position_y) //Write to the costmap the current object position
{ //It is written through the dynamic reconfigure 
  dynamic_reconfigure::Config conf;
  double_param.name = "object_x";
  double_param.value = position_x;
  conf.doubles.push_back(double_param);

  double_param.name = "object_y";
  double_param.value = position_y;
  conf.doubles.push_back(double_param);

  bool_param.name = "enabled" ;
  bool_param.value = true;
  conf.bools.push_back(bool_param);


  srv_req.config = conf;

  ros::service::call("/move_base/global_costmap/simple_layer_sub/set_parameters", srv_req, srv_resp);

  std_srvs::Empty empty;
  ros::service::call("/move_base/clear_costmaps", empty);

  ROS_INFO("Added to costmap and cleared costmaps");
  
}

void get_object_pose_callback(const geometry_msgs::PointStamped::ConstPtr& object_pose) //Send the initial object position through RVIZ, change topic
{
  
  geometry_msgs::Point objectpose = object_pose->point;
  std_msgs::Header head = object_pose->header;
  std::string frameid = "map";
  if( frameid == head.frame_id){
    x = objectpose.x;
    y = objectpose.y;
    ROS_INFO("Got object pose");
    write_to_costmap(x,y);
  }
  else
  {ROS_INFO("Wrong frame_id for object location");}
  
}



void object_goal_callback(const geometry_msgs::PoseStamped::ConstPtr& object_goal) //Get the object final position
{
  geometry_msgs::Pose objectgoal = object_goal->pose;
  std_msgs::Header head = object_goal->header;
  if( head.frame_id != "map" ) ROS_INFO("The goal was not in the map frame");
  else{
    x_goal = objectgoal.position.x;
    y_goal = objectgoal.position.y;
    goal_quat = objectgoal.orientation;
    ROS_INFO("Got goal of object");
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pushing_navigation_goals");
  ros::NodeHandle n("~");
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  
  ros::Subscriber act = n.subscribe("get_close", 10, start_move); //Just a topic for starting the action, debugging
  ros::Subscriber get_object_pose = n.subscribe("object_pose", 10, get_object_pose_callback); //Get pose of object
  ros::Subscriber object_goal = n.subscribe("object_goal", 10, object_goal_callback); //Get goal of object
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
