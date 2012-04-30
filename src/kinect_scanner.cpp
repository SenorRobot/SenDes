
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>


#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>

#define AVG_FACTOR 3

//#include <dynamic_reconfigure/server.h>
//#include <SenDes/tiltConfig.h>




double maxTilt = 31;
double minTilt = -31;

double avgangle = 0;

double volatile tiltVal;

tf::TransformBroadcaster *broadpointer;


/*
   void paramcallback(SenDes::tiltConfig &config, uint32_t level) {

   ROS_INFO("Updating tilt parameters");

   }*/

void tiltcallback(const sensor_msgs::Imu tilt){


	double angle= -1* tiltVal *3.14159/180/2;

	

	avgangle = (AVG_FACTOR * avgangle +atan2(tilt.linear_acceleration.y,tilt.linear_acceleration.z)-3.14159/2)/(AVG_FACTOR+1);

	broadpointer->sendTransform(
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0, angle, 0, 1), tf::Vector3(0.2, 0.0, 0.64)),
				ros::Time::now(),"base_link", "openni_camera"));

}

int main(int argc, char** argv){

	ros::init(argc, argv, "kinect_scanner");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	std_msgs::Float64 tilt;

	tf::TransformBroadcaster broadcaster;
	broadpointer=&broadcaster;
	
	ros::Publisher tilt_pub= n.advertise<std_msgs::Float64>("tilt_angle",1);


	ros::Subscriber sub= n.subscribe("imu",1,tiltcallback);


	/*dynamic_reconfigure::Server<SenDes::tiltConfig> server;
	  dynamic_reconfigure::Server<SenDes::tiltConfig>::CallbackType f;

	  f = boost::bind(&paramcallback, _1, _2);
	  server.setCallback(f);*/

	double rate = 1;
	double startTilt;
	pn.getParam("start_tilt",startTilt);
	pn.getParam("tilt_rate",rate);
	pn.getParam("max_tilt",maxTilt);
	pn.getParam("min_tilt",minTilt);

	tiltVal = startTilt;

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate r(10);

	while(n.ok()){



		tiltVal+=rate;
		if(tiltVal >= maxTilt){
			tiltVal=maxTilt;
			rate*=-1;
		}
		if(tiltVal <= minTilt){
			tiltVal=minTilt;
			rate*=-1;
		}

		current_time = ros::Time::now();

		tilt.data=tiltVal;
		//publish the message
		tilt_pub.publish(tilt);

		last_time = current_time;

		ros::spinOnce();
		r.sleep();
	}
}


