#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <cmath>

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_scan_publisher");

  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan_obstacle", 50);

  double laser_frequency = 40;
  bool last_bool = false;  

  int count = 100.0;
  ros::Rate r(10.0);
  while(n.ok()){
    int num_readings = 100;
    
    ros::Time scan_time = ros::Time::now();
    //Just testing 
    double x ;
    double y ;
    bool enabled;
    ros::param::param<bool>("/move_base/global_costmap/simple_layer_sub/enabled", enabled, false);
    if( enabled != last_bool)
    {
         last_bool = enabled;
         if(enabled) ROS_INFO("Fake scan enabled");
         else ROS_INFO("Fake scan disabled");
         
    }
    
    //ROS_INFO("Enabled ");
    else
    //ROS_INFO("Not enabled");
    ros::param::param<double>("/move_base/global_costmap/simple_layer_sub/object_x", x, 0.0);
    //ROS_INFO(" GOT x %f", x);
    ros::param::param<double>("/move_base/global_costmap/simple_layer_sub/object_y", y, 0.0);
    //ROS_INFO(" GOT y %f", y);
    
    //Converting to polar coordinates
    double p = sqrt(pow(x,2)+pow(y,2));
    double angle = atan2(y,x);
    int p_int = abs(p);
    if ( p_int > 0)  num_readings = num_readings * p_int;
    double ranges[num_readings];
    double intensities[num_readings];
    //generate some fake data for our laser scan
    for(unsigned int i = 0; i < num_readings; ++i){
      ranges[i] = 99.0;
      intensities[i] = 100 + count;
    }
    //ROS_INFO("Angle: %f", angle);
    //ROS_INFO("radius: %f", p);
    //Assining the two closest points
    int point_1 = 0, point_2 = 0;
    double closest_match = 0.0;
    double last_closest = 0.0;
    for(int i=0; i< num_readings; i++){
      double angle_test = -3.14 + i*(6.28/num_readings);
      //ROS_INFO("ANgle test %f",std::abs(angle_test - angle));
      if( std::abs(angle_test - angle) < std::abs(angle - closest_match)){
        last_closest = closest_match;
        point_2 = point_1;
        closest_match = angle_test;
        point_1 = i;
        //ROS_INFO("Got to this point");
 }
      else if( std::abs(angle_test - angle) < std::abs(angle - last_closest)) {
        last_closest = angle_test;
        point_2 = i; }
    }


    //populate the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "map";
    scan.angle_min = -3.14;
    scan.angle_max = 3.14;
    scan.angle_increment = 6.28 / num_readings;
    scan.time_increment = (1 / laser_frequency) / (num_readings);
    scan.range_min = 0.0;
    scan.range_max = 100.0;
    //ROS_INFO("Point 1 %i", point_1);
    //ROS_INFO("Point 2 %i", point_2);
    scan.ranges.resize(num_readings);
    //scan.intensities.resize(num_readings);
    for(unsigned int i = 0; i < num_readings; ++i){
      scan.ranges[i] = ranges[i];
      if( i == point_1 && enabled) scan.ranges[i] = p;
      else if( i==point_2 && enabled) scan.ranges[i] = p; 
      //scan.intensities[i] = intensities[i];
    }

    scan_pub.publish(scan);
    //++count;
    r.sleep();
  }
}
