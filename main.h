#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <modbus.h>
#include <modbus/modbus.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>

/* Temperature controller connection infomation */
#define BTC9100_CMB_PORTNAME 	"/dev/ttyUSB0"
#define BTC9100_MODBUS_PARITY 	'E' /* 'N' for none, 'E' for even, 'O' for odd */
#define BTC9100_CMB_STOPBITS 	1
#define BTC9100_CMB_DATABITS 	8 /* allow value are 5,6,7 and 8 */
#define BTC9100_MODBUS_BAUDRATE 9600
#define BTC9100_MODBUS_SLAVE_ID 2
#define BTC9100_SP1_REG_ADDR 	0
#define BTC9100_PV_REG_ADDR 	64

/* Controller connection information */
#define EZD305F_CMB_PORTNAME 	"/dev/ttyUSB0"
#define EZD305F_CMB_PARITY 		0   /* 0 = None, PARENB = Even, PARODD = Odd */
#define EZD305F_CMB_STOPBITS 	1 /* CSTOPB = 2 bits, otherwise = 1bits */
#define EZD305F_CMB_DATABITS 	CS8
#define EZD305F_CMB_BAUDRATE 	B9600 /* B + baudrate */

/* SlaveID with different EZD305F device */
#define FAN_EZD305F_SLAVE_ID  0x0A
#define BULB_EZD305F_SLAVE_ID 0x00

/* Command Define */
#define TEMP_READ 	0
#define TEMP_SETUP 	1
#define FAN_SETUP 	2
#define FAN_ON 		3
#define FAN_OFF 	4
#define BULB_SETUP 	5
#define BULB_ON 	6
#define BULB_OFF 	7

/* Web socket reference */
#define LISTEN_PORT 	1500
#define MAX_LISTEN 		5
#define MAX_DATA_LENS 	100

/* Static number */
#define TEMP_RANGE 	0.5

/* Communication Protocol */
/*
#define NOTHING		0x00
#define READ_DATA 	0x01
#define WRITE_DATA 	0x10
#define RETURN_DATA 0x11 
*/

/* Function */
static int cmdConvertToNum(char *);
static unsigned char* percentToHex(int);
static void tempRead();
static void tempSetup(float);
static void devSetup(unsigned char, int);
static void devOnOff(unsigned char, unsigned char);
static void *tempControl(void *tmp);
static void dealCommand(char *, char *);
static int openSerial();
static int closeSerial();
static int openModbus();
static int closeModbus();
static void openSocket();
static void listenFromSocket();
static unsigned char* percentToHex(int);
