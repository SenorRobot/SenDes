#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Range.h>



#include <linux/input.h>
#include <fcntl.h>
#include <pthread.h>

#define PI 3.14159
#define NUM_SONARS 6

int fd; //file descriptor
				        //x,y,z,r,p,y
double sonar_pos[NUM_SONARS][6] = {
					{0.0,-.2,0.2,0.0,0.0,-PI/2},
					{.14,-.14,0.2,0.0,0.0,-PI/4},
					{0.2,0.0,0.2,0.0,0.0,0.0},
					{.14,.14,0.2,0.0,0.0,PI/4},
					{0.0,0.2,0.2,0.0,0.0,PI/2},
					{-.2,.0,0.2,0.0,0.0,PI}};

char sonar_range[NUM_SONARS];




void *sonarThread (void* args){
	char sonar_string[NUM_SONARS];

	while(1){
		if(read(fd,sonar_string, sizeof(sonar_string))){
			for(int i= 0; i< NUM_SONARS;i++){
				sonar_range[i]=sonar_string[i];
			}			
		lseek(fd,0,SEEK_SET);

		}
		usleep(10000); //10 milliseconds. 10hz
	}
	return 0;
}


int main(int argc, char** argv){
	ros::init(argc, argv, "sonar_driver");

	ros::NodeHandle n;
	tf::TransformBroadcaster tf_broadcaster;
	
	ros::Publisher sonar_pub[NUM_SONARS];
	for (int i =0; i<NUM_SONARS;i++){
		char topic[30];
		sprintf(topic,"/sonar/sonar%d",i);
		sonar_pub[i]= n.advertise<sensor_msgs::Range>( topic, 50);
	}

	if((fd=open(argv[1],O_RDONLY))<0){
		perror("error opening sonar file");
		exit(1);
	}


	pthread_t input;
	pthread_create(&input , NULL, &sonarThread, NULL);

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate r(10);//publish at 10 hz

	while(n.ok()){

		current_time = ros::Time::now();

		for(int i =0; i< NUM_SONARS; i++){
			char sonar_frame[10];

			sprintf(sonar_frame,"/sonar%d",i);
			tf_broadcaster.sendTransform(
					tf::StampedTransform(
						tf::Transform(	tf::createQuaternionFromRPY(sonar_pos[i][3],sonar_pos[i][4],
								sonar_pos[i][5]),
							tf::Vector3(sonar_pos[i][0],sonar_pos[i][1],sonar_pos[i][2])),
						ros::Time::now(),"/base_link", sonar_frame ));

			sensor_msgs::Range sonar_msg;
			sonar_msg.radiation_type=0; //ULTRASOUND;
			sonar_msg.field_of_view=0.628318531; //36 degrees
			sonar_msg.min_range=0.1524; //6 inches
			sonar_msg.max_range=6.5; //256 inches
			sonar_msg.range=sonar_range[i]/39.3700787; //convert from inches to meters

			sonar_msg.header.frame_id=sonar_frame;		

			sonar_pub[i].publish(sonar_msg);

		}
		last_time = current_time;

		ros::spinOnce();
		r.sleep();
	}
}

