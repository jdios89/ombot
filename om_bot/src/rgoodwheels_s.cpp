#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include <sstream>
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include "geometry_msgs/Vector3.h"

double l1 = 0.16;   //m
double l2 = 0.1675; //m
double radius_wheel = 0.055; //m

double vellinearx;
double vellineary;
double velangularz;

double FL = 0, FR = 0, RR = 0, RL = 0;
double lFLpos = 0, lFRpos = 0, lRRpos =0, lRLpos=0;
double FLpos = 0, FRpos = 0, RRpos =0, RLpos=0;


void command_vel(const geometry_msgs::Twist::ConstPtr& vel)
{
	geometry_msgs::Vector3 linear, angular;
	linear = vel->linear;
	angular = vel->angular;
	//This function calculates the speeds of the wheels using the inverse kinematics
	//The speed will be given in rad/sec
	FL = ((linear.x) + (linear.y) - ((l1+l2) * angular.z)) / radius_wheel ; 
	FR = ((linear.x) - (linear.y) + ((l1+l2) * angular.z)) / radius_wheel ;	
	RL = ((linear.x) - (linear.y) - ((l1+l2) * angular.z)) / radius_wheel ;
	RR = ((linear.x) + (linear.y) + ((l1+l2) * angular.z)) / radius_wheel ; 
	//The control node will receive the desired speeds in RPM , so it is neccesary to 
	//do a units conversion, from rad/sec to RPM , multiplying by a factor of 60 / 2 pi = 9.5496
	FL = FL * 9.5496;
	FR = FR * 9.5496;
	RL = RL * 9.5496;
	RR = RR * 9.5496;
	//Then we set the maximun OUTPUTS we can have in RPM by the motors
	if(FL > 100) 
	{ FL = 100;}
	else if (FL < -100)
	{ FL = -100;}
	if(FR > 100) 
	{ FR = 100;}
	else if (FR < -100)
	{ FR = -100;}
	if(RR > 100) 
	{ RR = 100;}
	else if (RR < -100)
	{ RR = -100;}
	if(RL > 100) 
	{ RL = 100;}
	else if (RL < -100)
	{ RL = -100;}
	//THen we set an offset to send it through a string to the other node
	
}

int main(int argc, char** argv) {
ros::init(argc, argv, "myRobot_move_joint");
ros::NodeHandle n;
ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("b", 1);
ros::Rate loop_rate(100);
ros::Subscriber sub = n.subscribe("cmd_vel", 1, command_vel);
const double degree = M_PI/180;
double rot4 = 90;
double lsecs = ros::Time::now().toSec();

sensor_msgs::JointState joint_state;

joint_state.name.resize(4);
joint_state.position.resize(4);
joint_state.name[0] ="FR";
joint_state.name[1] ="FL";
joint_state.name[2] ="RR";
joint_state.name[3] ="RL";

while (ros::ok()) {
    //update joint_state

    FRpos = lFRpos + FR * ((lsecs - ros::Time::now().toSec())/60);
    FLpos = lFLpos + FL * ((lsecs - ros::Time::now().toSec())/60);
    RRpos = lRRpos + RR * ((lsecs - ros::Time::now().toSec())/60);
    RLpos = lRLpos + RL * ((lsecs - ros::Time::now().toSec())/60);
    joint_state.header.stamp = ros::Time::now();
    joint_state.position[0] = -FRpos * 2 * 3.1415;
    joint_state.position[1] = -FLpos * 2 * 3.1415;
    joint_state.position[2] = -RRpos * 2 * 3.1415;
    joint_state.position[3] = -RLpos * 2 * 3.1415;
    
    
    joint_pub.publish(joint_state);
    lFLpos = FLpos;
    lFRpos = FRpos;
    lRLpos = RLpos;
    lRRpos = RRpos;
    lsecs =ros::Time::now().toSec();
    ros::spinOnce();
    loop_rate.sleep();
}
return 0;
}
