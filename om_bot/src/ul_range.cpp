#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include <sstream>
#include <string>


char frameid[] = "/ul_ranger";

class ulrange
{
  public:
     ulrange();
  private:
     void get_range(const std_msgs::String::ConstPtr& msg); 
	 std::string str_;
     float distance_cm_;
     ros::NodeHandle nh_;
     ros::Subscriber sub_;
     ros::Publisher ul_range_;
     float last_distance_;
     float alpha_; //for low pass filter	 
};

ulrange::ulrange():
  last_distance_(0),
  distance_cm_(0),
  alpha_(0.1){
  sub_=nh_.subscribe("speeds", 10, &ulrange::get_range,this);    //Get the range data
  ul_range_ = nh_.advertise<sensor_msgs::Range>("ul_range", 5); 
  }

void ulrange::get_range(const std_msgs::String::ConstPtr& msg)
{
  str_ = msg->data.c_str();
  distance_cm_ = ((int)((unsigned char)str_.at(4)));
  sensor_msgs::Range range_data;
  range_data.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_data.header.frame_id = frameid;
  range_data.field_of_view = 0.4;
  range_data.min_range = 0.03;
  range_data.max_range = 1;
  range_data.range = (distance_cm_/100) * alpha_ + (1.0 - alpha_) * (last_distance_); 
  last_distance_ = range_data.range;
  range_data.header.stamp = ros::Time::now();
  ul_range_.publish(range_data);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ulrange");
  
  ulrange ul_r;
    
  ros::Rate loop_rate(300);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

