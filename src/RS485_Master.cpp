

#include "USBSERIAL_Linux.h"

#include "util.h"

#include <stdio.h>
#include <pthread.h>
#include <iostream>
#include <signal.h>

using namespace std;

#define DEVICE_INDEX 1

extern sensorRecord sensor;

bool quit = false;
int addr = 5;

#define ALARM_INTERVAL_IN_S     (1)



static void sighand(int sig)
{
  static unsigned int last;

  switch (sig) {
  case SIGINT:
	  cout << "SIN INT" << endl;
	  quit = true;
	  break;
  case SIGALRM:
	  cout << "SIN ALARM" << endl;
	  alarm(ALARM_INTERVAL_IN_S);
	  break;
  }
}
int main(int argc, char *argv[]) {
//	printf("hello hasan \n");


	char dev[256] = "/dev/ttyUSB";

	double val1, val2;
	unsigned int id;

	if(argc > DEVICE_INDEX)
		strcpy(dev, argv[DEVICE_INDEX]);
	else {
		int deviceNo = MY_LIB::GetFirstDeviceNo(dev, 0, 20);
		if(deviceNo == -1)
			deviceNo = 0;

		sprintf(dev, "%s%d", dev, deviceNo);


	}

	KWSA_DEBUG("Connecting to device %s ... ", dev);


	/* Use sighand as our signal handler */
	//signal(SIGALRM, sighand);
	signal(SIGINT, sighand);
	//alarm(ALARM_INTERVAL_IN_S);

	/* Allow signals to interrupt syscalls */
	siginterrupt(SIGINT, 1);


	pthread_t handle;
   	if (pthread_create(&handle, NULL, USB2SERIAL_LINUX, (char*)dev) != 0) {

   		KWSA_DEBUG("Error in creating thread \n");
   		return -1;
   	}


   	static unsigned int expected = 0;

   	while(quit == false) {

		if(SensorRead(&id, &val1, &val2) == SENSOR_TRUE)
			KWSA_INFO("(%4d   %.2f   %.2f ) \n", id, val1, val2);
		else
			if(sensor.fd >= 0)
				KWSA_DEBUG("No data \n");

		if(id != expected)
			KWSA_ERROR("!!!!!!!!!!!!!!!!  Failed to get Expected ID !!!!!!!!!!!!!!!!!!1 \n");
		expected = id + 1;

   		sleep(1);

   	}

   	void *status;
   	pthread_join(handle, &status);

   	KWSA_DEBUG("Exiting... \n");

   	return 0;
}
