

#include "USBSERIAL_Linux.h"

#include "util.h"

#include <stdio.h>
#include <pthread.h>
#include <iostream>
#include <signal.h>

using namespace std;


#define KWSA_DEBUG(...)


extern sensorRecord sensor;

bool quit = false;
uint8_t addr = 5;

uint8_t nodeList[MAX_NO_OF_NODES] = {addr };
int nNodes = 1;

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

	bool devArgFound = false;

	double val1, val2;
	unsigned int count, sender;

	int argIndex = 1;
	uint8_t curAddr;
	int i;
	while(argIndex < argc) {

		if(argv[argIndex][0] >= '0' && argv[argIndex][0] <= '9') {
			if(argIndex == 1) {
				nNodes = 0;
			}
			curAddr = atoi(argv[argIndex]);
			for(i=0; i<nNodes; i++) {
				if(curAddr == nodeList[i]) {
					printf("Addr format error !!! \n");
					exit(1);
				}
			}

			nodeList[nNodes] = curAddr;
			nNodes ++;

		}
		else {
			strcpy(dev, argv[argIndex]);
			devArgFound = true;
			break;
		}
		argIndex ++;

	}

	if(devArgFound == false) {
		int deviceNo = MY_LIB::GetFirstDeviceNo(dev, 0, 20);
		if(deviceNo == -1)
			deviceNo = 0;

		sprintf(dev, "%s%d", dev, deviceNo);


	}

	for (i=0; i<nNodes; i++)
		printf("%d ", nodeList[i]);

	printf("\n");

	printf("%s \n", dev);

	//exit(0);
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

		if(SensorRead(&count, &sender, &val1, &val2) == SENSOR_TRUE) {
			KWSA_INFO("(sender addr: %2d %4d   %.2f   %.2f ) \n", sender, count, val1, val2);

			if(count != expected)
				;//KWSA_ERROR("!!!!!!!!!!!!!!!!  Failed to get Expected ID (%d)!!!!!!!!!!!!!!!!!!1 \n", expected);

			if(sender == nodeList[nNodes-1])
				expected = count + 1;

		}
		else
			if(sensor.fd >= 0)
				KWSA_DEBUG("No data \n");


   		sleep(1);

   	}

   	void *status;
   	pthread_join(handle, &status);

   	KWSA_DEBUG("Exiting... \n");

   	return 0;
}
