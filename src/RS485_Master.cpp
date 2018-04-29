#include "USBSERIAL_Linux.h"

#include <stdio.h>
#include <pthread.h>
#include <iostream>

using namespace std;

extern sensorRecord sensor;

char key = '0';
int addr = 5;



int main(int argc, char *argv[]) {
//	printf("hello hasan \n");

	char dev[256] = "/dev/ttyUSB0";

	double val1, val2;
	unsigned int id;

	if(argc > 1)
		strcpy(dev, argv[1]);

	pthread_t handle;
   	if (pthread_create(&handle, NULL, USB2SERIAL_LINUX, (char*)dev) != 0) {

   		KWSA_DEBUG("Error in creating thread \n");
   		return -1;
   	}


   	while(key != 'q') {

   		printf("Enter a key : ");
   		cin >> key;

   		if(key == 's' && sensor.ownPtr != NULL) {
   			if(SensorRead(&id, &val1, &val2) == SENSOR_TRUE)
   				KWSA_DEBUG("(%d   %.2f   %.2f \n)", id, val1, val2);
   		}

   	}

   	void *status;
   	pthread_join(handle, &status);

   	return 0;
}
