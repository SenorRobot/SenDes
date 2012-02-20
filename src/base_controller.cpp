#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <fstream>

#define BASE_RADIUS 0.1524 	//base radius of 6 inches
#define MPS_TO_PWM  128.0	//calibration constant. Meters per second to duty cycle

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel);

using namespace std;

fstream motor1, motor2;

int main (int argc, char ** argv){


	motor1.open("/dev/motor1");
	motor2.open("/dev/motor2");
	if(motor1.fail() || motor2.fail()){
		cout<< "error motor 1"<<endl;
	}


	ros::init(argc, argv, "base_controller");
	ros::Subscriber cmd_vel_sub;
	ros::NodeHandle node;
	cmd_vel_sub = node.subscribe<geometry_msgs::Twist>("/cmd_vel", 1,       boost::bind(cmdVelReceived,_1));
	ros::spin();


}
void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{

	//output to the motors at a calibrated rate to conform to the command
	std::cout << "x=" << cmd_vel->linear.x << std::endl;
	std::cout << "y=" << cmd_vel->linear.y << std::endl;
	std::cout << "theta=" << cmd_vel->angular.z << std::endl;

	double m1 = cmd_vel->linear.x ;
	double m2 = cmd_vel->linear.x ;

	double theta = cmd_vel->angular.z;


	m1 += theta * BASE_RADIUS;
	m2 -= theta * BASE_RADIUS;
	
	m1*= MPS_TO_PWM;
	m2*= MPS_TO_PWM;

	cout<<"m1:"<<(int)m1<<"    m2:"<<(int)m2<<endl;
	
	motor1 <<(char)m1;
	motor2 <<(char)m2; 

}
