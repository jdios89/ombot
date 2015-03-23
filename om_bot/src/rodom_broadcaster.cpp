#include <ros/ros.h>
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <string>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

double FLinput = 0, FRinput = 0, RRinput =0, RLinput=0;
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;
double l1 = 0.164; //0.156445; //m
double l2 = 0.1665; //0.16637; //m
double radius_wheel = 0.052; // 0.05146; //m
double last_vel = 0;
double last_velsp = 0;

std::string str;
//function to  get the speeds from the subscribed topic in rpm
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

	//Conversion from rpm to rad/sec with the factor 2 pi / 60 = 0.104717
	RRinput = RRinput * 0.104717;
	RLinput = RLinput * 0.104717;
	FRinput = FRinput * 0.104717;
	FLinput = FLinput * 0.104717;

	//Calculating the direct kinematics of the robot
	/*
	vx = R/4 * (FL + FR + RL + RR)
	vy = R/4 * (FL - FR - RL + RR)
	vth = R/4 * ((1/(-l1-l2))*FL + (1/(l1+l2))*FR + (1/(-l1-l2))*RL + (1/(l1+l2))*RR )
	*/
	vx = (radius_wheel/4) * (FLinput + FRinput + RLinput + RRinput);
	vy = (radius_wheel/4) * (-FLinput + FRinput +RLinput - RRinput);
/* Replaced by the IMU readings */
        vth = (radius_wheel/4) * ((1/(-l1-l2))*FLinput + (1/(l1+l2))*FRinput + (1/(-l1-l2))*RLinput + (1/(l1+l2))*RRinput );       
        vth = 0.984 * vth ; 
        last_vel = ros::Time::now().toSec(); 
}

void imu_yaw(const sensor_msgs::Imu::ConstPtr& imu_data)
{
  geometry_msgs::Vector3 angular;
  angular= imu_data->angular_velocity;  
 // vth = angular.z;
  last_velsp = ros::Time::now().toSec();
  
}


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("speeds", 10, getinspeeds);
  ros::Subscriber sub_imu = n.subscribe("Imu", 10, imu_yaw); 
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 20);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
  
 

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(100.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
   // if(last_velsp - ros::Time::now().toSec() > 0.5)
   // {
   //     vx = 0;
   //     vy = 0; 
  //  }
  //  if(last_vel - ros::Time::now().toSec() > 0.5)
  //  {
  //      vth = 0;
  //  }
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z =-0.0662;// 0.0662;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0812779;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
