#include "USBSERIAL_Linux.h""

#include <stdio.h>
#include <pthread.h>



int main(int argc, char *argv[]) {
//	printf("hello hasan \n");

	char dev[256] = "/dev/ttyUSB0";

	if(argc > 1)
		strcpy(dev, argv[1]);

	pthread_t handle;
   	if (pthread_create(&handle, NULL, USB2SERIAL_LINUX, (char*)dev) != 0) {

   		KWSA_DEBUG("Error in creating thread \n");
   		return -1;
   	}

   	char key = '0';
   	while(key != 'q') {
   		scanf("%c", &key);

   	}
}
