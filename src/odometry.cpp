#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>



tf::TransformBroadcaster* odom_broadcaster_p;
void republishcallback(const nav_msgs::Odometry odom){

	//since all odometry is 6DOF we'll need a quaternion created from yaw
	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = odom.header.stamp;
	odom_trans.header.frame_id = odom.header.frame_id;
	odom_trans.child_frame_id = odom.child_frame_id;

	odom_trans.transform.translation.x = odom.pose.pose.position.x;
	odom_trans.transform.translation.y = odom.pose.pose.position.y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom.pose.pose.orientation;
		
	//send the transform
	odom_broadcaster_p->sendTransform(odom_trans);

	printf(".");

}

int main(int argc, char** argv){
	ros::init(argc, argv, "odometry_rebroadcaster");

	ros::NodeHandle n;


	tf::TransformBroadcaster odom_broadcaster;
	odom_broadcaster_p = &odom_broadcaster;

	ros::Subscriber sub= n.subscribe("odom",1,republishcallback);

	ros::Rate r(100);
	while(n.ok()){
		ros::spinOnce();
		r.sleep();
	}
}
