#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <my_repository/get_closeAction.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <cmath>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

float input_range = 0.0; //Initialize the input range with arbitrary data
float setpoint_range = 0.13; //Its in meters !
float output_speed_x = 0.0;
float kp = -0.55; //negative to set inverse direction
float ki = -0.07; //negative
float kd = -0.0; //negative
float outMin = -0.14;
float outMax = 0.14;
float error;
float ITerm;
float last_range;

class get_closeAction
{
protected:

  bool success;
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<my_repository::get_closeAction> as_;
  std::string action_name_;
  float goal_;
  ros::Subscriber sub_; //get the range sensor data
  ros::Publisher cmd_vel_; //Publish the speed commanded
  // create messages that are used to published feedback/result
  my_repository::get_closeFeedback feedback_;
  my_repository::get_closeResult result_;

public:

  get_closeAction(std::string name) :
      as_(nh_, name, boost::bind(&get_closeAction::executeCB,this, _1), false),
    action_name_(name)
  {
      //register the goal and feeback callbacks
      //as_.registerGoalCallback(boost::bind(&get_closeAction::executeCB, this));
      as_.registerPreemptCallback(boost::bind(&get_closeAction::preemptCB, this));
      //Publish to the data of interest
      cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",10);
      //subscribe to the data topic of interest
      sub_ = nh_.subscribe("/ulrange", 10, &get_closeAction::get_rangeCB, this);
      error = 0;
      ITerm = 0;
      last_range = 0;
      as_.start();
  }

  ~get_closeAction(void)
  {
  }
  /*
  void goalCB()
  {
    // reset helper variables
    ITerm = 0;
    last_range = 0;
    error = 0;
    // accept the new goal
    goal_ = as_.acceptNewGoal()->distance;
    setpoint_range = goal_;
  }
*/
  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void get_rangeCB(const sensor_msgs::Range::ConstPtr& range)
  {
    input_range = range->range;
    //if(activated){}
    //ROS_INFO("Got this range %f", input_range);
  //  distance_cmt = str.at(9) << 8 | str.at(10);
  }

  void executeCB(const my_repository::get_closeGoalConstPtr &goal)
  //void executeCB()
  {
    // make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;
    //goal_ = as_.acceptNewGoal()->distance;
    goal_ = goal->distance;
    setpoint_range = goal_;
    success = false;
    ros::Rate loop_rate(100.0);
    while(ros::ok() && as_.isActive() && !as_.isPreemptRequested() && !success)
    {
        //input_range = range->range;
        feedback_.distance_from = input_range;
        as_.publishFeedback(feedback_);

        float DTerm;
        float error;
        float KTerm;
        error = setpoint_range - input_range;
        ROS_INFO("error %f ", error);
        float sampletime = 1/100.0;
        ROS_INFO("sampletime %f ",sampletime);
        
        ITerm += ki * error * sampletime;
        ROS_INFO("iterm %f ", ITerm);
        
        if(ITerm > outMax) ITerm = outMax;
        else if(ITerm < outMin) ITerm = outMin;
        DTerm = kd * (input_range - last_range) / sampletime ;
        ROS_INFO("dterm %f ", DTerm);
        
        KTerm = kp * error;
        ROS_INFO("KTerm %f ", KTerm);
        
        last_range = input_range;
        output_speed_x = KTerm + ITerm + DTerm;

        if(output_speed_x > outMax) output_speed_x = outMax;
        else if(output_speed_x < outMin) output_speed_x = outMin;
        geometry_msgs::Twist command_vel;
        
        if(std::abs(input_range - goal_) < 0.03)
        {
            result_.final_distance = feedback_.distance_from;
            result_.result = "Success";
            as_.setSucceeded(result_);
            success = true;
            command_vel.linear.x = 0.0;
            cmd_vel_.publish(command_vel);
        }
        else
        {
            command_vel.linear.x = output_speed_x;
            cmd_vel_.publish(command_vel);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    if (as_.isPreemptRequested())
    {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        //Set the action state to preempted
        as_.setPreempted();
        success = false;
    }
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_close");

  get_closeAction get_close(ros::this_node::getName());
  ros::spin();

  return 0;
}
