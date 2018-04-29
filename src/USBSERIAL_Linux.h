#ifndef USBSERIAL_LINUX_H_
#define USBSERIAL_LINUX_H_


#include <pthread.h>

#include "ace/Log_Msg.h"



#define STX 'X'
#define ETX 'Y'

#define INVALID_HANDLE 1
#define ERROR_WRONG_SETTING 2
#define WRITE_ERROR 3
#define WAIT_EVENT_ERROR 4

#define	BAUD_NO	6	//Anzahl moeglicher Baudraten


#define SENSOR_OK 0
#define SENSOR_ERROR -1
#define SENSOR_TRUE 1

#define	SLEEP_IN_THREAD	1
#define DATA_KEY 0xA5
#define	DATA_POSTFIX_1	0x0D
#define	DATA_POSTFIX_2	0x0A
#define PARAM_KEY 0x3B

#define	FRAME_SIZE 10
#define PURGE_LOOP_MAX 15	/* Schleifenzaehler f. PurgeComm-Loop, Zeit: *TIMEOUT_PURGE ms */

#define MUTEX_MAX_WAITTIME 20000//Wartezeit fuer WaitSingleObject(io_mutex)
#define TIMEOUT_PURGE 100	/* Wartezeit beim Abbruch, bis erneuter Abbruch versucht wird */
#define WRITE_TIMEOUT_MULTIPLIER	2	//min Bus-Speed: 9600Bd, 10 Bits/Byte: 10000/9600 <2 ms pro byte
#define WRITE_TIMEOUT_CONSTANT	100	//in ms, wichtig wg.�Schnittstellen-failure, dh ploetzlicher Komm.-Abbruch



#define OWN_ERR_MASK 0x20000000 //Bit29=1, s.http://msdn.microsoft.com/en-us/library/ms680347(VS.85).aspx
#define ERR_MUTEXFAILED 0x300000F0	//d805306608
#define ERR_EVENTFAILED 0x300000F1	//d805306609
#define ERR_PAR_NO 0x300000F2		//d805306610
#define ERR_MEM_ALLOC	0x300000F3	//d805306611 eventuell zuwenig RAM
#define ERR_NO_GSV4_FOUND 0x300000F4	//d805306612 Comport konnte zwar geoeffnet werden, aber kein GSV4 antwortete
#define ERR_BYTES_WRITTEN 0x300000F5	//d805306613
#define ERR_WRONG_PARAMETER 0x30000100	//d805306624
#define ERR_NO_ANSWER 0x30000058	//d805306456
#define ERR_WRONG_ANSWER_NUM 0x30000059	//d805306457
#define ERR_WRONG_ANSWER 0x30000060	//d805306458
#define ERR_WRONG_FRAME_SUFFIX	0x30000061
#define ERR_NOT_SUPPORTED 0x30000062	//Firmware-Version zu alt
#define ERR_NO_OWNER 0x30000101	//d805306625
#define ERR_COM_ALREADY_OPEN	0x300000F6	//d805306614
#define ERR_COM_GEN_FAILURE 0x3000001F //hardware-or driver-error of COMport. See: ERROR_GEN_FAILURE @msdn.microsoft.com/en-us/library/ms681382(VS.85).aspx

#define IN_BUFSIZE	32768 //Windows-buffer incoming
#define OUT_BUFSIZE 256	//Windows-buffer outgoing
#define IN_OWNBUFSIZE	256	//Ablage fuer empfangene Bytes
#define PARAM_INBUF_SIZE 128 //fuer Parameterempfang


#define ACTEX_FLAG_HANDSHAKE	4	//enables HW-handshake in in GSV4activateExtended
#define ACTEX_FLAG_WAIT_EXTENDED	0x0100	//waits longer for device-answer in GSV4activateExtended
#define ACTEX_FLAG_STOP_TX	0x0200	//stops continious data transmission in GSV4activateExtended


#define RESPONSEFRAME_FIXEDSIZE 10	//Parameterantwortframe
#define PARAMRESPONSE_TIMEOUT 6000 //((10000/9600) * 50 * RESPONSEFRAME_FIXEDSIZE) // in ms
#define TIMEOUT_PURGE 100	/* Wartezeit beim Abbruch, bis erneuter Abbruch versucht wird */
#define PURGE_LOOP_MAX 15	/* Schleifenzaehler f. PurgeComm-Loop, Zeit: *TIMEOUT_PURGE ms */
#define STACK_SIZE 65536   /* Stapelgr��e f�r "Hintergrund"-Thread */
#define MUTEX_MAX_WAITTIME 20000//Wartezeit fuer WaitSingleObject(io_mutex)
//#define RELEASE_WAIT_TIME 5000	//Wartezeit fuer GSV4release
#define WAIT_AFTER_STOPTX	200	//Wartezeit nach StopTx-Kommando innerhalb der Befehlsausfuehrung
#define MAX_BUS_LOAD 0.96	//maximale zeitliche Auslastung d. Schnittstellenbusses W: 0.1..1
#define WAIT_ACTIVATE 500 //Wartezeit am Ende der GSV4actExt in ms
#define WAIT_ACTIVATE_FACT 4	//mit ACTEX_FLAG_WAIT_EXTENDED: Wartezeit= WAIT_ACTIVATE*WAIT_ACTIVATE_FACT
#define TRIALS_GET_CMD 5	////mit ACTEX_FLAG_WAIT_EXTENDED: Anz. d. getMode-versuche in GSV4actExt
#define TIMEOUT_GSV_INIT 3000U   /* Erweitertes Timeout zB bei Handshake */
#define TIMEOUT_POLLFLOW 100U   /* Intervall f�r Abfrage der Hardware-Flow-Control */
#define DELAY_BETWEEN_COMMANDS	80	//in ms, bei Schleifenaufrufen


#define INVALID_FILE_FD 1000
#define READ_FAILED 1001
#define GET_COM_ATTR_ERROR 1002
#define SET_COM_ATTR_ERROR 1003

#define THREAD_CREATE_ERROR 1004
#define TTY_WRITE_ERROR 1005


#define ABORT_ACTIVATE \
	SensorRelease; \
   return -1


#define CHECK_OWNER \
	 if (sensor.ownPtr==NULL) \
     {	\
		sensor.LastError= ERR_NO_OWNER;	\
		return SENSOR_ERROR;	\
	 }

#define START_TX \
	if(sensor.txOn) \
	{			\
		if(StartTX(ComNo)==SENSOR_ERROR) \
			return SENSOR_ERROR; \
	}


#define RES_GETLASTERROR \
	{	\
		sensor.LastError= GetLastError() | OWN_ERR_MASK; \
		res= SENSOR_ERROR; \
	}
#define RES_CHK_BYTES_WRITTEN(no) \
	else if(Byteswritten != (no))	\
	{	\
		sensor.LastError= ERR_BYTES_WRITTEN; \
		res= SENSOR_ERROR;	\
	}


#define RES_MUTEXFAILED	\
	{	\
		sensor.LastError= ERR_MUTEXFAILED; \
		res= SENSOR_ERROR; \
	}

#define WRITE_BYTES_CHK_SUCC(num)	\
	if((Byteswritten=write(sensor.fd, txbuf, (num)))!= (num)) \
	{	\
		sensor.LastError= TTY_WRITE_ERROR; \
		res= SENSOR_ERROR; \
	}


typedef bool BOOL;
typedef unsigned int DWORD;

typedef struct _sensordata
{
	volatile unsigned int id;
	volatile unsigned long value1;
	volatile unsigned long value2;
	volatile unsigned char status; //[4];   /* Status*/
} sensordata;

struct sensorRecord	//fuer je 1 GSV-4 mit CHAN_NO=4 Kanaelen
{
	volatile void *ownPtr;

	unsigned long timeouts;
	int fd;
	long Bitrate;
	unsigned char *in_buffer;   /* Lesepuffer (Zeiger) */
								//unsigned long in_size;
	unsigned long in_pos;   /* Lesepuffer aktuelle Lese-Position */
	unsigned long in_count;   /* Lesepuffer Anzahl Bytes */
	unsigned long in_missing;   /* Lesepuffer Anzahl fehlende Bytes, */
								/* um einen Datenrecord oder Parameterrecord vervollst�ndigen zu k�nnen */
	pthread_t thread; //HANDLE thread;

	int terminate;	//=1: Thread beenden
	int is_terminated;
	pthread_mutex_t	io_mutex;	//f. Messwerte lesen (Zugriff auf out_buffer)
	pthread_mutex_t Prd_mutex;	//f. Parameter lesen (Zugriff auf Param_in_buf)
	pthread_cond_t Prd_cond;
	int  Prd_event;	//Signal: Alle (Lese-) Parameter fertig empfangen
	pthread_mutex_t Clr_mutex; //Signal: in_buffer geloescht
	pthread_cond_t Clr_cond;
	int Clr_event;	//Signal: in_buffer geloescht
						//gsvdata LastValue[CHAN_NO];
	sensordata *out_buffer;   /* Lesepuffer (Zeiger) */
	unsigned int out_size;
	volatile unsigned int out_put;   /* Datenpuffer Einf�geposition */
	volatile unsigned int out_get;   /* Datenpuffer Entnahmeposition */
	volatile unsigned int out_count;   /* Datenpuffer Datensatzanzahl */
	volatile unsigned char* Param_in_buf;	//Ablage f. Read-Parameter
											//volatile unsigned long Param_in_pos;
	volatile unsigned int Param_in_count;
	volatile unsigned int Param_in_missing;
	volatile DWORD error;   /* Ablage f�r Fehler-Code, write-Zugriff NUR im Lesethread*/
	BOOL clear;	//=1: Buffer loeschen
	BOOL txOn; //=1: Staendige Messwertuebertragung des GSV ist an
	unsigned int FWversion;	//MERGSV$ v.1.1.: Firmware-Version speichern (wenn kommuniziert), sonst =0
	volatile unsigned int LastError;	//jetzt Member, write-Zugriff in Fkt, NICHT im Thread


	sensorRecord();

};

void move_data(sensorRecord *tmp);




void * USB2SERIAL_LINUX(void *ptr);

void * data_pump(void *ptr);

int SensorStopTX();
int StopTX();
int SensorGetMode();
int SensorGetTxMode();
double CalcData(const unsigned int val);
int SensorGetValue();
int SensorRead(unsigned int * id, double* val1, double* val2);

int SensorActivate(const char * dev, long Bitrate, long BufSize, long flags);
int SensorRelease();


#define DEBUG 0
#define INFO 1
#define WARN 2
#define ERROR 3
#define FATAL 4

#define V_LEVEL 0
#ifndef V_LEVEL
#error
#endif




#if V_LEVEL <= DEBUG
#define KWSA_DEBUG(...) ACE_DEBUG((LM_DEBUG, __VA_ARGS__))
#define _DEBUG_
#else
#define KWSA_DEBUG(...)
#endif

#if V_LEVEL <= INFO
#define KWSA_INFO(...) ACE_DEBUG((LM_DEBUG, __VA_ARGS__))
#else
#define KWSA_INFO(...)
#endif

#if V_LEVEL <= WARN
#define KWSA_WARN(...) ACE_DEBUG((LM_DEBUG, __VA_ARGS__))
#else
#define KWSA_WARN(...)
#endif

#if V_LEVEL <= ERROR
#define KWSA_ERROR(...) ACE_DEBUG((LM_DEBUG, __VA_ARGS__))
#else
#define KWSA_ERROR(...)
#endif

#if V_LEVEL <= FATAL
#define KWSA_FATAL(...) ACE_DEBUG((LM_DEBUG, __VA_ARGS__))
#else
#define KWSA_FATAL(...)
#endif




#define KWSA_RETURN(X, Y) \
	do { \
		X; \
		return Y; \
	} while (0)



#define KWSA_DEBUG_RETURN(X, Y) \
	do { \
		KWSA_RETURN(SKI_ERROR X, Y);\
	} while (0)


#define KWSA_ERROR_RETURN(X, Y) \
	do { \
		KWSA_RETURN(SKI_ERROR X, Y);\
	} while (0)


//#define SKI_ERROR_RETURN TEST_DEBUG1

#define KWSA_REQUIRE_RETURN(exp, ret) do{ if (!(exp)){ SKI_WARN(#exp "\n"); return (ret); }}while(0)

#define KWSA_ASSERT ACE_ASSERT

#define KWSA_NEW_RETURN ACE_NEW_RETURN




#endif
