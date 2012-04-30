#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Range.h>



#include <linux/input.h>
#include <fcntl.h>
#include <pthread.h>

#define PI 3.14159
#define NUM_SONARS 6

int fd; //file descriptor
				        //x,y,z,x,y,z,w
double sonar_pos[NUM_SONARS][7] = {
					{0,0,0,0,0,-PI,1},
					{0,0,0,0,0,-PI/2,1},
					{0,0,0,0,0,0,1},
					{0,0,0,0,0,PI/2,1},
					{0,0,0,0,0,PI,1},
					{0,0,0,0,0,2*PI,1}
}

char sonar_range[NUM_SONARS];




void *sonarThread (void* args){
	char sonar_string[NUM_SONARS*2-1];//one char and one comma per reading, minus fencepost

	while(1){
		if(read(fd,sonar_string, sizeof(sonar_string))){
			for(int i= 0; i< NUM_SONARS;i++){
				sonar_range[i]=sonar_string[i*2];
			}			

		}
		usleep(10000); //10 milliseconds. 10hz
	}
	return 0;
}


int main(int argc, char** argv){
	ros::init(argc, argv, "odometry_publisher");

	ros::NodeHandle n;
	ros::Publisher  = n.advertise<>("sonar", 50);
	tf::TransformBroadcaster tf_broadcaster;



	double vx = 0;
	double vy = 0;
	double vth = 0;

	if((fd=open(argv[1],O_RDONLY))<0){
		perror("error opening sonar file");
		exit(1);
	}


	pthread_t input;
	pthread_create(&input , NULL, &inputThread, NULL);

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate r(10);//publish at 10 hz

	while(n.ok()){

		current_time = ros::Time::now();

		for(int i =0; i< NUM_SONAR; i++){
			char sonar_frame[10];

			sprintf(sonar_frame,"sonar[%d]",i);
			tf_broadcaster.sendTransform(
					tf::StampedTransform(
						tf::Transform(	tf::Quaternion(sonar_pos[i][0],sonar_pos[i][1],
								sonar_pos[i][2],sonar_pos[i][3]),
							tf::Vector3(sonar_pos[i][4],sonar_pos[i][5],sonar_pos[i][6])),
						ros::Time::now(),"base_link", sonar_frame ));
		}
		last_time = current_time;

		ros::spinOnce();
		r.sleep();
	}
}

