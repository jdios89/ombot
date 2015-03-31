#include <ros/ros.h>
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
int looprate = 100;
float sampletime = 1/(float)looprate ;
 
bool activated = false;
void get_range(const sensor_msgs::Range::ConstPtr& range)
{
  input_range = range->range;
  if(activated){}
  //ROS_INFO("Got this range %f", input_range);
//  distance_cmt = str.at(9) << 8 | str.at(10);
}
void get_close(const std_msgs::Bool::ConstPtr& activation)
{
  activated = activation->data;
} 
int main(int argc, char **argv)
{

  ros::init(argc, argv, "get_close_object");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",10);    //Publish speed commands
  ros::Publisher achieved = n.advertise<std_msgs::String>("get_close_status",10);
  ros::Subscriber sub = n.subscribe("ulrange", 100, get_range);
  ros::Subscriber act = n.subscribe("get_close", 10, get_close);
  ros::Rate loop_rate(looprate);
  while (ros::ok())
  {
    if(activated){
    float DTerm;
    float error;
    float KTerm;
    error = setpoint_range - input_range;
    
    ITerm += ki * error * sampletime;
    if(ITerm > outMax) ITerm = outMax;
    else if(ITerm < outMin) ITerm = outMin;
    DTerm = kd * (input_range - last_range) / sampletime ;
    KTerm = kp * error;
    last_range = input_range;
    output_speed_x = KTerm + ITerm + DTerm;
    
    if(output_speed_x > outMax) output_speed_x = outMax;
    else if(output_speed_x < outMin) output_speed_x = outMin;
    geometry_msgs::Twist command_vel;
    //ROS_INFO("Publishing this speed %f",output_speed_x);
    if(std::abs(error) < 0.02){
    output_speed_x = 0.0;
    activated = false;
    std::string message = "Got close at ";
    std::ostringstream ss;
    ss << input_range;
    std::string s(ss.str());
    message += s;
    message += "cm";
    std_msgs::String message_achieved;
    message_achieved.data = message;
    achieved.publish(message_achieved);
    }  
    command_vel.linear.x = output_speed_x;
    
    pub.publish(command_vel);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

