#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <sstream>
#include "turtlesim/TeleportAbsolute.h"
#include "turtlesim/Pose.h"
#include "math.h"
#include "geometry_msgs/Vector3.h"
//Dimensions of the robot
double l1 = 0.16;   //m
double l2 = 0.1665; // 0.16637; //m
double radius_wheel = 0.052; //0.05146; //m
double FL = 0, FR = 0, RR = 0, RL = 0;
char datum[4];
std_msgs::String msg;


class Kinematics
{
   public:
     Kinematics();
	 double timesec(); 
	 void publish_null();
     	
   private:
     
	 ros::NodeHandle nh_;
	 ros::Subscriber sub_; 
	 ros::Publisher myString_;
	 double secs;
         void command_vel_(const geometry_msgs::Twist::ConstPtr& vel);

};

Kinematics::Kinematics():
       secs(0){
	     sub_ = nh_.subscribe("cmd_vel", 10, &Kinematics::command_vel_, this);
             myString_ = nh_.advertise<std_msgs::String>("desSpeed", 10);//Advertise to the control node
	   }
double Kinematics::timesec()
{
   return secs;
}
void Kinematics::publish_null()
{	 
	   datum[0] = (char)((int) 101);
	   datum[1] = (char)((int) 101);
	   datum[2] = (char)((int) 101);
	   datum[3] = (char)((int) 101);
	   msg.data = datum;
	   myString_.publish(msg);
}
		
//cmd_vel topic 
void Kinematics::command_vel_(const geometry_msgs::Twist::ConstPtr& vel)
{
	geometry_msgs::Vector3 linear, angular;
	linear = vel->linear;
	angular = vel->angular;
	//This function calculates the speeds of the wheels using the inverse kinematics
	//The speed will be given in rad/sec
	//FL = ((linear.x) + (linear.y) - ((l1+l2) * angular.z)) / radius_wheel ; 
	//FR = ((linear.x) - (linear.y) + ((l1+l2) * angular.z)) / radius_wheel ;	
	//RL = ((linear.x) - (linear.y) - ((l1+l2) * angular.z)) / radius_wheel ;
	//RR = ((linear.x) + (linear.y) + ((l1+l2) * angular.z)) / radius_wheel ; 
	FL = ((linear.x) - (linear.y) - ((l1+l2) * angular.z)) / radius_wheel ; 
        FR = ((linear.x) + (linear.y) + ((l1+l2) * angular.z)) / radius_wheel ;       
        RL = ((linear.x) + (linear.y) - ((l1+l2) * angular.z)) / radius_wheel ;
        RR = ((linear.x) - (linear.y) + ((l1+l2) * angular.z)) / radius_wheel ; 

        //The control node will receive the desired speeds in RPM , so it is neccesary to 
	//do a units conversion, from rad/sec to RPM , multiplying by a factor of 60 / 2 pi = 9.5496
	FL = FL * 9.5496;
	FR = FR * 9.5496;
	RL = RL * 9.5496;
	RR = RR * 9.5496;
	//Saturation filter - we set the maximun OUTPUTS we can have in RPM by the motors 
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
	FL = FL + 101;
	FR = FR + 101;
	RR = RR + 101;
	RL = RL + 101;
	//We convert them to integers and then to characters to be sent
	datum[0] = (char)((int) FL);
	datum[1] = (char)((int) FR);
	datum[2] = (char)((int) RR);
	datum[3] = (char)((int) RL);
	//Preparing the message to be sent
	msg.data = datum;
	myString_.publish(msg);
	secs = ros::Time::now().toSec() + 1;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Kinematics");
        ros::start();
	ros::Rate loop_rate(300);
	Kinematics kinematics;
	while (ros::ok())
	{
		if(ros::Time::now().toSec() > kinematics.timesec())  //If no speeds are written in more than one second then send zero speed
		{
                   kinematics.publish_null();
		}
		ros::spinOnce();
		loop_rate.sleep();
	}	
	return 0;
}
