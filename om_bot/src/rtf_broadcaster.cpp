#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_tf_publisher");
	ros::NodeHandle n;

	ros::Rate r(150);

	tf::TransformBroadcaster broadcaster;
	tf::TransformBroadcaster broadcaster2;

	while(n.ok())
	{
		broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(-0.2665, 0.0, 0.2915)), ros::Time::now(), "base_link", "camera_depth_frame"));
		//broadcaster2.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(-0.2665, 0.0, 0.2915)), ros::Time::now(), "odom", "camera_depth_frame"));
		r.sleep();
	}
}


