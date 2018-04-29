/*
 * USBSERIAL_Linux.cpp
 *
 *  Created on: Apr 26, 2018
 *      Author: usrc
 */


#include "USBSERIAL_Linux.h"

#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <limits.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <string.h>
#include <stdio.h>
#include <termios.h>
#include <iostream>
#include <assert.h>


extern char key;
extern int addr;

bool quit = false;



static	long baud_table[] =
{
	//4800,
	9600,  19200, 38400, 57600, 115200, 230400
};

static unsigned long LastErrorGlobal;	//Errorcode, nur (noch) fuer GSV4actExt

sensorRecord sensor;

static tcflag_t BaudrateToBaudrateCode( unsigned long baudrate );

//static int ConditionTimeout(pthread_cond_t *pCondition, pthread_mutex_t *pMutex, int * pEvent, unsigned long timoutUs);

static double CalcData(const unsigned long val);
static void set_blocking (int fd, int should_block);
static int set_interface_attribs (int fd, int speed, int parity);

sensorRecord::sensorRecord()
{

	KWSA_ASSERT(pthread_mutex_init(&io_mutex, NULL) == 0);
	KWSA_ASSERT(pthread_mutex_init(&Prd_mutex, NULL) == 0);
	KWSA_ASSERT(pthread_cond_init(&Prd_cond, NULL) == 0);
	KWSA_ASSERT(pthread_cond_init(&Clr_cond, NULL) == 0);
	fd = 0;
	ownPtr = NULL;

}


void * USB2SERIAL_LINUX(void *ptr) {

	char * dev = (char *) ptr;

	int ret = SensorActivate(dev, 9600, 256, 0);

	if(ret != SENSOR_OK)
		KWSA_DEBUG("USB2SERIAL_LINUX: ret is %d Global error is %d \n", ret, LastErrorGlobal);
	while (quit == false) {

		switch(key) {
			case 'x':
				SensorGetValue();
				key = '0';
				break;
			case 'q':
				quit = true;
				break;
		}

		usleep(1000000);
	}

	SensorRelease();
	return NULL;
}


void * data_pump(void *ptr) //war static void
{
	sensorRecord *tmp = (sensorRecord *)ptr;   /* Zeiger auf GSV-Datensatz */
	BOOL res;
	DWORD err;

	/* Solange keine Beendigung gew�nscht */
	while (!tmp->terminate)
	{
		tmp->in_count = 0;


		if (!tmp->in_missing)
			tmp->in_missing = 1;

		/* Lesen */
		/*
			res = ReadFile(tmp->hcom, tmp->in_buffer + tmp->in_pos,
				tmp->in_missing, &tmp->in_count, NULL); //&tmp->action_read);
				*/

		tmp->in_count = read(tmp->fd, tmp->in_buffer + tmp->in_pos,
							 tmp->in_missing); //&tmp->action_read);



		if(!tmp->in_count)	{//wenn nix gelesen: -> if not read:
#ifdef SLEEP_IN_THREAD

			tmp->error = READ_FAILED;
			usleep(SLEEP_IN_THREAD*1000);	//Suspend the execution of the current thread
#endif
		}

		//err = res ? ERROR_SUCCESS : GetLastError();
		else
		{
			 /* Wenn Buffer-Clear gefordert -> When Buffer-Clear is required*/


			pthread_mutex_lock(&tmp->Clr_mutex);
			if (tmp->clear)
			{
				/* Lesepuffer löschen */
				tmp->in_count = 0;
				tmp->in_pos = 0;
				tmp->in_missing = 0;

				/* Löschanforderung zurücksetzen -> Clear the deletion request */
				tmp->clear = false;

				tmp->Clr_event = 1;

				pthread_cond_signal(&tmp->Clr_cond);

				tmp->error= 0;	//evtl. alten fehler loeschen

				pthread_mutex_unlock(&tmp->Clr_mutex);

			}
			else
			{
				pthread_mutex_unlock(&tmp->Clr_mutex);

				/* Anzahl um vorher bereits vorhandene Anzahl erhöhen -> Increase number by previously existing number*/
				tmp->in_count += tmp->in_pos;
				/* Wieder von ganz vorne beginnen -> Start again from the very beginning*/
				tmp->in_pos = 0;

				/* Eingegangene Daten analysieren -> Analyze incoming data*/
				if(tmp->in_count)	{//runtime-optim.: nur call, wenn was gelesen -> Runtime-optimized .: only call if what read
					move_data(tmp);

				}
			}


		}

	}
	if (tmp->terminate)	//TODO: Testen: ok
	{
		/* Thread beenden */
		KWSA_DEBUG("data_pump: Terminated \n");
		tmp->is_terminated=1;

	}
	return NULL;
}

void move_data(sensorRecord *tmp)
{
	unsigned long j;
	unsigned char *p;

	j = tmp->in_count;   /* Anzahl Bytes */
	p = tmp->in_buffer;   /* Hier starten */
	tmp->in_missing = 0;   /* Resultat voreinstellen auf 0 */

	while (j)
	{
		sensordata t;
		unsigned long inx;
		char suffix[2];


		if (*p == DATA_KEY)
		{
			/* Sind gen�gend Daten da, um den Record vollst�ndig zu verarbeiten ? */
			if (j < FRAME_SIZE)
			{
				/* Wenn nein, Abbrechen und fehlende Byte-Anzahl angeben */
				tmp->in_missing = FRAME_SIZE - j;
				break;
			}

			/* Restanzahl berechnen */
			j -= FRAME_SIZE;
			p++;

			p++;

			memcpy((void *)&t.id, p, sizeof(unsigned int));
			p += sizeof(unsigned int);
			memset((void *)&t.value1, 0, sizeof(t.value1));

			int k = sizeof(t.value1);
			*((unsigned char *)&t.value1 + 3) = 0;
			*((unsigned char *)&t.value1 + 2) = 0;
			*((unsigned char *)&t.value1 + 1) = (p[0]);
			*((unsigned char *)&t.value1 + 0) = (p[1]);

			p += 2;

			*((unsigned char *)&t.value2 + 3) = 0;
			*((unsigned char *)&t.value2 + 2) = 0;
			*((unsigned char *)&t.value2 + 1) = (p[0]);
			*((unsigned char *)&t.value2 + 0) = (p[1]);

			/* gesch�tzten Daten-Lese-Bereich betreten */
			if(pthread_mutex_lock(&tmp->io_mutex) == 0) {
					/* �berspringen, wenn Buffer clear gefordert */
				if (!tmp->clear)
				{
					/* Wert und Status in Datenpuffer legen und */
					/* Zeiger etc. anpassen */

					if (tmp->out_count < tmp->out_size)
						tmp->out_count++;
					else				//Buffer voll: alte Werte ueberschreiben
						tmp->out_get = (tmp->out_get + 1) % tmp->out_size;

					*(tmp->out_buffer+tmp->out_put)= t;
					//tmp->out_buffer[tmp->out_put].value1 = t.value1;
					tmp->out_put = (tmp->out_put + 1) % tmp->out_size;

				}
				pthread_mutex_unlock(&tmp->io_mutex);
							  /* geschützten Daten-Lese-Bereich verlassen */


				if (tmp->clear)
					return;

			}
			else
				tmp->error = ERR_MUTEXFAILED;
		}
		/* sonstige Zeichen */
		else
		{
			/* ignorieren */
			p++;
			j--;
#ifdef _DEBUG
			Beep(2000, 50);
#endif
		}
	}	//end while (j)
		/* unverarbeiteten Rest an den Anfang des Puffers kopieren */
		/* und Position zum Anf�gen setzen */
	if (j)
	{
		memmove(tmp->in_buffer, p, j);
		tmp->in_pos = j;
	}
}




int SensorActivate(const char * dev, long Bitrate, long BufSize, long flags) {

	int i, ret, mode;
	unsigned int max_s, bufsizetmp;
	BOOL baudok = false;
	char portstr[16];
	//unsigned int Timeout;

	termios io_set_old, io_set_new;

	unsigned int wtime;

	sensorRecord *tmpGSV = NULL;
							/* Grunds�tzliche Zul�ssigkeit */
	/* Datenpuffer: Gr��e �berpr�fen und anlegen */
	max_s = (int)(((DWORD)-1) / FRAME_SIZE);

	if (BufSize<1)
		BufSize = 1;
	else if (BufSize > max_s)
		bufsizetmp = max_s;
	else bufsizetmp = BufSize;

	for (i = 0; i<BAUD_NO; i++)	//baudrate checken
	{
		if (baud_table[i] == Bitrate)
		{
			baudok = true;
			break;
		}
	}
	if (!baudok)
		//	RET_WRONG_PARAMETER;
	{
		LastErrorGlobal = ERR_WRONG_PARAMETER;
		return SENSOR_ERROR;
	}

	if (sensor.fd != 0 || sensor.ownPtr != NULL)
	{
		LastErrorGlobal = ERR_COM_ALREADY_OPEN;
		return SENSOR_ERROR; 	//Schnittstelle bereits geoeffnet
	}

	tmpGSV = &sensor;

	tmpGSV->in_buffer = (unsigned char *)malloc(IN_OWNBUFSIZE);

	if (tmpGSV->in_buffer == NULL)
	{
		LastErrorGlobal = ERR_MEM_ALLOC;
		ABORT_ACTIVATE;
	}
	//tmpGSV->in_missing = 0;
	tmpGSV->out_size = bufsizetmp; // * sizeof(long);

	tmpGSV->out_buffer = (sensordata*)malloc(tmpGSV->out_size * sizeof(sensordata));

	if (!tmpGSV->out_buffer)
	{
		LastErrorGlobal = ERR_MEM_ALLOC;
		ABORT_ACTIVATE;
	}
		//tmpGSV->out_put[i]=0; tmpGSV->out_get[i]=0;

	tmpGSV->Param_in_buf = (unsigned char*)malloc(PARAM_INBUF_SIZE);	//buffer fuer Read-Parameter
	if (!tmpGSV->Param_in_buf)
	{
		LastErrorGlobal = ERR_MEM_ALLOC;
		ABORT_ACTIVATE;
	}
	tmpGSV->Param_in_missing = 0;	//wichtig (eig. red. wg calloc)

	tmpGSV->fd= open (dev, O_RDWR | O_NOCTTY | O_SYNC);//open(dev, O_RDWR);


	if(tmpGSV->fd < 0)
	{
		LastErrorGlobal= INVALID_FILE_FD;
		ABORT_ACTIVATE;
	}

	set_interface_attribs (tmpGSV->fd, B9600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (tmpGSV->fd, 1);                // set no blocking

	/*

	if (tcgetattr( tmpGSV->fd, &io_set_old ) < 0)
	{
		LastErrorGlobal = GET_COM_ATTR_ERROR;
		ABORT_ACTIVATE;
	}

	io_set_new=io_set_old;


	KWSA_ASSERT(BaudrateToBaudrateCode( Bitrate ));

	io_set_new.c_cflag |=  CLOCAL;    // set Local line - do not change "owner" of port
	io_set_new.c_cflag |=  HUPCL;     // Hangup (drop DTR) on last close
	io_set_new.c_cflag |=  CREAD;     // enable reader
	io_set_new.c_cflag &= ~PARENB;    // set 8N1
	io_set_new.c_cflag &= ~CSTOPB;
	io_set_new.c_cflag &= ~CSIZE;
	io_set_new.c_cflag |=  CS8;
	io_set_new.c_cflag &= ~CRTSCTS;   // disable hardware flow control
	io_set_new.c_cflag &= ~CBAUD;     // clear baudrate bits first
	io_set_new.c_cflag |=  BaudrateToBaudrateCode( Bitrate ); // set requested baudrate bits
	io_set_new.c_cflag &=  ~ICRNL;
	io_set_new.c_cflag &=  ~INLCR;
	io_set_new.c_cflag &=  ~IGNBRK;
	io_set_new.c_cflag |=  BRKINT;
	io_set_new.c_cflag &=  ~IGNPAR;
	io_set_new.c_cflag |=  INPCK;
	io_set_new.c_cflag |=  ISTRIP;
	io_set_new.c_cflag |=  IGNCR;

	io_set_new.c_cflag &=  ~CRTSCTS;

	io_set_new.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                      | INLCR | IGNCR | ICRNL | IXON);
	io_set_new.c_oflag &= ~OPOST;
	io_set_new.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	io_set_new.c_cflag &= ~(CSIZE | PARENB);
	io_set_new.c_cflag |= CS8;


	if (tcsetattr(tmpGSV->fd,TCSANOW,&io_set_new) < 0)
	{
		LastErrorGlobal = SET_COM_ATTR_ERROR;
		ABORT_ACTIVATE;
	}

	*/
	tmpGSV->terminate = false;
   	tmpGSV->clear = false;

   	if (pthread_create(&tmpGSV->thread, NULL, data_pump, (sensorRecord*)&sensor) != 0) {

   		LastErrorGlobal = THREAD_CREATE_ERROR;
   		ABORT_ACTIVATE;
   	}
	/*thread-init war _beginthread(gsv_data_pump, STACK_SIZE, &gsvs[ComNo])*/

   	tmpGSV->Bitrate = Bitrate;

   	sensor.ownPtr= &sensor;

	if(flags & ACTEX_FLAG_WAIT_EXTENDED)
		wtime= WAIT_ACTIVATE * WAIT_ACTIVATE_FACT;
	else
		wtime= WAIT_ACTIVATE;
	usleep(wtime*1000);


	  /* Test auf Kommunikationsbereitschaft */
	if (flags & ACTEX_FLAG_HANDSHAKE)
	{
		KWSA_ASSERT(0);

	}

	if (flags & ACTEX_FLAG_STOP_TX)
	{
		if (SensorStopTX() == SENSOR_ERROR)
		{
			LastErrorGlobal = sensor.LastError; //LastError retten
			SensorRelease();
			return SENSOR_ERROR;
		}
		if (flags & ACTEX_FLAG_WAIT_EXTENDED)
			usleep(WAIT_AFTER_STOPTX*1000);
	}
	else
		sensor.txOn = true;	//Annehmen, staendige Datenuebertrgung sei an


	sensor.LastError = 0;
	LastErrorGlobal = 0;

	/*ERstzer Schnittstellen Schreib- und Lesezugriff*/

	if (flags & ACTEX_FLAG_WAIT_EXTENDED)
	{

		for (i = 0; i<TRIALS_GET_CMD; i++)
		{
			mode = SensorGetMode();
			if (mode != SENSOR_ERROR)
				break;
		}
		if (mode == SENSOR_ERROR)
		{
			LastErrorGlobal = sensor.LastError;	//LastError retten

			SensorRelease();	//kein GSV4 gefunden
			return SENSOR_ERROR;

		}
	}
	else
	{
		if ((mode = SensorGetMode()) == SENSOR_ERROR)
		{
			LastErrorGlobal = sensor.LastError;	//wird innerhalb der Fkt gesetzt

			SensorRelease();	//kein GSV4 gefunden
								//LastErrorGlobal= ERR_NO_GSV4_FOUND;	//
			return SENSOR_ERROR;

		}
	}

	ret = SensorGetTxMode();
	if (ret == SENSOR_ERROR)
	{
		if (sensor.LastError != ERR_NO_ANSWER)	//ganz alte FW kennt kein GetTXmode: kein Error
		{
			LastErrorGlobal = sensor.LastError;	//wird innerhalb der Fkt gesetzt
			SensorRelease();	//kein GSV4 gefunden
								//LastErrorGlobal= ERR_NO_GSV4_FOUND;	//
			return SENSOR_ERROR;
		}
	}
	else
	{
		if (ret & 0x02)	//Bit1: aktueller TX-state
			sensor.txOn = true;
		else
			sensor.txOn = false;
	}

	return SENSOR_OK;

}

int SensorGetValue() {

	char txbuf[256];
	DWORD Byteswritten;
	int res = SENSOR_OK;

	CHECK_OWNER;
	memset(txbuf, 0, sizeof(txbuf));

	sprintf(txbuf, "%cGET VALUE %03d%c", STX, addr, ETX);



	//WRITE_BYTES_CHK_SUCC(16);

	int len = strlen(txbuf);

	if((Byteswritten=write(sensor.fd, txbuf, (len)))!= (len))
	{
		sensor.LastError= TTY_WRITE_ERROR;
		res= SENSOR_ERROR;
	}



	return res;

}


tcflag_t BaudrateToBaudrateCode( unsigned long baudrate )
{
    switch (baudrate)
    {
#ifdef B3000000
    case 3000000: return B3000000;
#endif
#ifdef B2500000
    case 2500000: return B2500000;
#endif
#ifdef B2000000
    case 2000000: return B2000000;
#endif
#ifdef B1500000
    case 1500000: return B1500000;
#endif
#ifdef B1152000
    case 1152000: return B1152000;
#endif
#ifdef B1000000
    case 1000000: return B1000000;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
#ifdef B576000
    case 576000: return B576000;
#endif
#ifdef B500000
    case 500000: return B500000;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B256000
    case 256000: return B256000;
#endif
#ifdef B230400
    case 230400: return B230400;
#endif
    case 115200: return B115200;
    case 57600:  return B57600;
    case 38400:  return B38400;
    case 19200:  return B19200;
    case 9600:   return B9600;
    case 4800:   return B4800;
    case 2400:   return B2400;
    case 1800:   return B1800;
    case 1200:   return B1200;
    case 600:    return B600;
    case 300:    return B300;
    case 200:    return B200;
    case 150:    return B150;
    case 134:    return B134;
    case 110:    return B110;
    case 75:     return B75;
    case 50:     return B50;
    }

    return 0;
}

int SensorStopTX()
{
	int ret = SENSOR_OK;

	ret = StopTX();

	if (ret == SENSOR_OK)
	{
		sensor.txOn = false;
		return SENSOR_OK;
	}
	else
		return SENSOR_ERROR;
}

int StopTX()
{
	char txbuf[2];
	DWORD Byteswritten;

	CHECK_OWNER;
	txbuf[0] = 0x23;

	if(write(sensor.fd, txbuf, 1) != 0)
	{
		sensor.LastError = ERR_BYTES_WRITTEN | OWN_ERR_MASK;
		return SENSOR_ERROR;
	}

	if (Byteswritten != 1)
	{
		sensor.LastError = ERR_BYTES_WRITTEN;
		return SENSOR_ERROR;
	}
	return SENSOR_OK;
}


int SensorGetMode() {
	return SENSOR_OK;
}

int SensorGetTxMode() {
	return SENSOR_OK;
}

int SensorRead(unsigned int *id, double* out1, double* out2)
{
	int result = SENSOR_TRUE, ix; //,err;

	CHECK_OWNER;

	sensorRecord *tmp = (sensorRecord *)sensor.ownPtr; //(gsvrecord *) redundant?




	if (pthread_mutex_lock(&tmp->io_mutex) == 0) //if(EnterGSVreadMutex(ComNo))
	{
		//gsvrecord *tmp = (gsvrecord *)gsvs[ComNo].ownPtr; //(gsvrecord *) redundant?
		if (!tmp->out_count)	//hat dieser Kanal DAten?
			result = SENSOR_OK;			//nein: result=0
		if (result && !tmp->error)
		{
			*id = tmp->out_buffer[tmp->out_get].id;
			*out1 = CalcData(tmp->out_buffer[tmp->out_get].value1);
			*out2 = CalcData(tmp->out_buffer[tmp->out_get].value2);
			tmp->out_get = (tmp->out_get + 1) % tmp->out_size;
			tmp->out_count--;
			KWSA_DEBUG("%d \n", tmp->out_count);
		}
		pthread_mutex_unlock(&tmp->io_mutex);
	}
	else
	{
		tmp->LastError = ERR_MUTEXFAILED;
		result = SENSOR_ERROR;
	}
	if (tmp->error)
		result =SENSOR_ERROR;
	return result;
}


int SensorRelease()
{

    CHECK_OWNER;




	if(sensor.ownPtr == NULL)
		return SENSOR_OK;

	sensor.Prd_event = 0;


	sensor.terminate =1;

	void *status;


	if (sensor.thread != 0 )	{
		sensor.clear = true;   /* Auch Buffer-Clear setzen zur Vereinfachung */

		if (sensor.fd)  { /* Einlesevorgänge abbrechen */
			tcflush(sensor.fd, TCOFLUSH);
			close(sensor.fd);
			sensor.fd = 0;
		}

	/* MUTEXs freigeben, da Arrayeinträge bereits leer sind, */
		 /* gibt es nichts mehr zu schützen */
		//pthread_mutex_unlock(&gsvs[nodeId].io_mutex);



		pthread_join(sensor.thread, &status);


	}

	sensor.thread = 0;
		 	/* Thread endgültig schließen */
	/* nicht mehr benötigte Events schließen */



	if(sensor.in_buffer)
	{
		free(sensor.in_buffer);

		sensor.in_buffer=NULL;


	}



	if(sensor.out_buffer)
	{
		free(sensor.out_buffer);
		sensor.out_buffer=NULL;
	}


	sensor.ownPtr= NULL;

	return SENSOR_OK;

}

double CalcData(const unsigned long val) {

	printf("value is %ld \n", val);
	double ret =  ((double)val*0.0382 + 3.6165);

	return ret;
}


int
set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
        	KWSA_ERROR ("error %d from tcgetattr", errno);
            return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
        	KWSA_ERROR ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void
set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                KWSA_ERROR ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        	KWSA_ERROR ("error %d setting term attributes", errno);
}
