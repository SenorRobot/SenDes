#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "basic_transform_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
	  broadcaster.sendTransform(
			  tf::StampedTransform(
				  tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, -0.2, 0.64)),
				  ros::Time::now(),"base_link", "openni_camera"));

	  broadcaster.sendTransform(
			  tf::StampedTransform(
				  tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.2, 0.0, 1.05)),
				  ros::Time::now(),"base_link", "wifiAntenna"));
	  r.sleep();
  }
}

