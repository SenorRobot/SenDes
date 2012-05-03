#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <fstream>

#define BASE_RADIUS 0.12 	//base radius of 6 inches in m
#define MPS_TO_PWM  256.0	//calibration constant. Meters per second to duty cycle

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel);

using namespace std;

fstream motor1, motor2;

int main (int argc, char ** argv){


	motor1.open("/dev/motor_r");
	motor2.open("/dev/motor_l");
	if(motor1.fail() || motor2.fail()){
		cout<< "error opening file"<<endl;
	}

	ros::init(argc, argv, "base_controller");
	ros::Subscriber cmd_vel_sub;
	ros::NodeHandle node;
	cmd_vel_sub = node.subscribe<geometry_msgs::Twist>("/cmd_vel", 1,       boost::bind(cmdVelReceived,_1));
	ros::spin();


}
double lastm1=0;
double lastm2=0;

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{

	//output to the motors at a calibrated rate to conform to the command
	std::cout << "x=" << cmd_vel->linear.x << std::endl;
	std::cout << "y=" << cmd_vel->linear.y << std::endl;
	std::cout << "theta=" << cmd_vel->angular.z << std::endl;

	double m1 = cmd_vel->linear.x/2; //scale this down, because 1m/s is really fast
	double m2 = cmd_vel->linear.x/2;


	double theta = cmd_vel->angular.z; //in radians

	m1 -= theta * BASE_RADIUS;
	m2 += theta * BASE_RADIUS;
	
	m1*= MPS_TO_PWM;
	m2*= MPS_TO_PWM;
	
	if (m1 > 127) m1=127;
	if (m1 < -127) m1=-127;

	if (m2 > 127) m2=127;
	if (m2 < -127) m2=-127;

	m2 *=-1; //negate because one motor is backwards..

	cout<<"m1:"<<(int)m1<<"    m2:"<<(int)m2<<endl;

	motor1 <<(int8_t)m1;
	motor2 <<(int8_t)m2; 

	motor1.flush();
	motor2.flush();

	lastm1=m1;
	lastm2=m2;

}
