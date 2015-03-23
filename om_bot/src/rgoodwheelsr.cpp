#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include <sstream>
double FLinput = 0, FRinput = 0, RRinput =0, RLinput=0;
double lFLpos = 0, lFRpos = 0, lRRpos =0, lRLpos=0;
double FLpos = 0, FRpos = 0, RRpos =0, RLpos=0;
std::string str, str2;

void getinspeeds(const std_msgs::String::ConstPtr& msg)
{
	int temp;
	str = msg->data.c_str();
	temp = ((int)((unsigned char)str.at(0)));
	RRinput = (float)temp - 120;   //RR
	temp = ((int)((unsigned char)str.at(1)));
	RLinput = (float)temp - 120;   //RL
	temp = ((int)((unsigned char)str.at(2)));
	FRinput = (float)temp - 120;    //FR
	temp = ((int)((unsigned char)str.at(3)));
	FLinput = (float)temp - 120;  //FL

	if(abs(FLinput) <=2)
	{
		FLinput = 0;
	}
	if(abs(FRinput) <=2)
	{
		FRinput = 0;
	}
	if(abs(RRinput) <=2)
	{
		RRinput = 0;
	}
	if(abs(RLinput) <=2)
	{
		RLinput = 0;
	}
}

int main(int argc, char** argv) {
ros::init(argc, argv, "myRobot_move_joint");
ros::NodeHandle n;
ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("b", 1);
ros::Rate loop_rate(100);
ros::Subscriber sub = n.subscribe("speeds", 100, getinspeeds);
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

    FRpos = lFRpos + FRinput * ((lsecs - ros::Time::now().toSec())/60);
    FLpos = lFLpos + FLinput * ((lsecs - ros::Time::now().toSec())/60);
    RRpos = lRRpos + RRinput * ((lsecs - ros::Time::now().toSec())/60);
    RLpos = lRLpos + RLinput * ((lsecs - ros::Time::now().toSec())/60);
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
