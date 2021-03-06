#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <dynamic_reconfigure/server.h>
#include <SenDes/mouseOdometryConfig.h>


#include <linux/input.h>
#include <fcntl.h>
#include <pthread.h>

#include <fstream>
//#define RADIUS .15 //15 cm

double x = 0.0, lastx = 0.0;
double y = 0.0, lasty = 0.0;

double pos = 0.0;
double lastpos = 0.0;

volatile double th = 0.0, lastth=0.0;

int fd; //file descriptor
FILE* yawfd;

char yawfile[40];

struct input_event ev;
typedef struct {
	int32_t xVal;
} gyro_event;

volatile double linearCalibration;
volatile double angularCalibration=0.0;


double radius;

void callback(SenDes::mouseOdometryConfig &config, uint32_t level) {
	linearCalibration=config.linearCalibration;
	angularCalibration=config.angularCalibration;
	radius=config.radius;
	ROS_INFO("Updating parameters, angular:%f",angularCalibration);

}
void *inputThread (void* args){

	while ( 1 ){
		if(read(fd,&ev,sizeof(struct input_event))){
			if(ev.type==2){
				switch (ev.code){
					case 0: //mouse x event
						pos+= ev.value/linearCalibration;
						
						//th+=ev.value/linearCalibration/radius * sin(angularCalibration);
						break;
					case 1:
						//mouse y event
						//pos+= ev.value/linearCalibration *sin(angularCalibration);
						//th+=ev.value/(linearCalibration)/radius * cos(angularCalibration);

						break;
					case 2:
						break;
				}
			}
		}
		usleep(10);
	}
	return 0;
}

void *gyroThread (void* args){
	int32_t yaw=0;
	while(1){
		if(fscanf(yawfd,"%d",&yaw)){	
			th=-yaw/10000.0*angularCalibration;	
			fseek(yawfd,0,SEEK_SET);
		}

		usleep(100);
	}
	return 0;
}


int main(int argc, char** argv){
	ros::init(argc, argv, "odometry_publisher");

	ros::NodeHandle n;
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
	tf::TransformBroadcaster odom_broadcaster;

	dynamic_reconfigure::Server<SenDes::mouseOdometryConfig> server;
	dynamic_reconfigure::Server<SenDes::mouseOdometryConfig>::CallbackType f;

	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);


	if (!n.getParam("linearCalibration",  (double& ) linearCalibration)){
		ROS_INFO("Using default linear calibration");
		linearCalibration=29500.0;
		n.setParam("linearCalibration", (double ) linearCalibration);
	}

	if (!n.getParam("angularCalibration", (double&)angularCalibration)){
		ROS_INFO("Using default angular calibration");
		angularCalibration=1.2;
		n.setParam("angularCalibration",(double)angularCalibration);
	}
	if (!n.getParam("radius", radius)){
		ROS_INFO("Using default radius");
		radius=.15;
		n.setParam("radius",radius);
	}


	double vx = 0;
	double vy = 0;
	double vth = 0;

	if((fd=open(argv[1],O_RDONLY))<0){
		perror("Mouse Device already open");
		exit(1);
	}
	if((yawfd=fopen(argv[2],"r"))<0){
		perror("gyro Device already open");
		exit(1);
	}

	strncpy(yawfile,argv[2],40);

	pthread_t input;
	pthread_create(&input , NULL, &inputThread, NULL);

	pthread_t gyro;
	pthread_create(&gyro, NULL, &gyroThread, NULL);

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate r(1000);

	while(n.ok()){

		current_time = ros::Time::now();

		//compute odometry in a typical way given the velocities of the robot
		double dt = (current_time - last_time).toSec();


		double delta_x = ((pos-lastpos) * cos(th) );
		double delta_y = ((pos-lastpos) * sin(th) );

		//double delta_th = vth * dt;

		vx = 0.0;//(x-lastx)/dt;
		vy = 0.0;//(y-lasty)/dt;
		vth= (th-lastth)/dt;


		x += delta_x;
		y += delta_y;

		lastx=x;
		lasty=y;
		lastth=th;
		lastpos=pos;

		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "/odom";
		odom_trans.child_frame_id = "/base_link";

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
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
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;

		//publish the message
		odom_pub.publish(odom);

		last_time = current_time;

		ros::spinOnce();
		r.sleep();
	}
}
