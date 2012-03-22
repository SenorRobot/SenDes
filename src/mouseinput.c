#include <stdio.h>

#include <linux/input.h>
#include <fcntl.h>
#include <stdlib.h>

int main (int argc, char ** argv){

	int fd; //file descriptor
	if((fd=open("/dev/input/event8",O_RDONLY))<0){
		perror("device already open");
		exit(1);
	}

	int x=0,y=0;

	struct input_event ev;
	while(1){
		read(fd,&ev,sizeof(struct input_event));
		//printf("value %d, type %d, code %d\r\n",ev.value,ev.type,ev.code);
		if(ev.type==2){

			switch (ev.code){
				case 0:
					x+=ev.value;
					break;
				case 1:
					y+=ev.value;
					break;
				case 2:
					break;
			}

		}

		printf("x: %d y:%d\r\n",x,y);
	}
}
