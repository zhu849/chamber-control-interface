#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

/* Temperature controller RS232 connection infomation */
#define BTC9100_CMB_PORTNAME	"/dev/ttyUSB0"
#define BTC9100_CMB_PARITY		PARENB	/* 0 = None, PARENB = Even, PARODD = Odd */
#define BTC9100_CMB_STOPBITS	1		/* CSTOPB = 2 bits, otherwise = 1bits */
#define BTC9100_CMB_DATABITS	8
#define BTC9100_CMB_BAUDRATE	B9600	/* B + baudrate */
#define BTC9100_SLAVE_ID		"2"

/* Fan controller RS232 connection information */
#define EZD305F_CMB_PORTNAME	"/dev/ttyUSB0"
#define EZD305F_CMB_PARITY		0	   /* 0 = None, PARENB = Even, PARODD = Odd */
#define EZD305F_CMB_STOPBITS	1	   /* CSTOPB = 2 bits, otherwise = 1bits */
#define EZD305F_CMB_DATABITS	8
#define EZD305F_CMB_BAUDRATE	B9600  /* B + baudrate */
#define EZD305F_SLAVE_ID		"0A"

/* Static Number */
#define MAX_STR_LENS 50

/* Command Define */
#define TEMP_READ	0
#define TEMP_SETUP	1
#define FAN_SETUP	2
#define FAN_ON		3
#define FAN_OFF		4
#define BULB_SETUP	5
#define BULB_ON		6
#define BULB_OFF	7

/* Global variable */
int comport;//File descriptor
struct termios newPortSetting, oldPortSetting;

/* Function */
static int cmdConvertToNum(char*);
static void tempRead();
static void tempSetup();
static void fanSetup();
static void fanOn();
static void fanOff();
static void bulbSetup();
static void bulbOn();
static void bulbOff();
static int setConnectionInfo();

/* Convert command of string type to corresponding number */  
int cmdConvertToNum(char *str) {
	if (strcmp(str, "TempRead")) return TEMP_READ;
	else if(strcmp(str, "TempSetup")) return TEMP_SETUP;
	else if (strcmp(str, "FanSetup")) return FAN_SETUP;
	else if(strcmp(str, "FanOn")) return FAN_ON;
	else if (strcmp(str, "FanOff")) return FAN_OFF;
	else if (strcmp(str, "BulbOn")) return BULB_ON;
	else if (strcmp(str, "BulbOff")) return BULB_OFF;
}

/* Set Information about serial port connection */
int setConnectionInfo() {
	int status;/* Record ioctl message */

	comport = open(BTC9100_CMB_PORTNAME, O_RDWR | O_NOCTTY | O_NDELAY); /* Open file can read&write in nonblocking mode */
	if (comport == -1) {
		printf("Unable to open comport with %s\n", BTC9100_CMB_PORTNAME);
		return 0;
	}
	
	/* Get parameters associated with the object referred by fd and stores them in the termios structure.*/
	if (tcgetattr(comport, &oldPortSetting) == -1) {
		close(comport);
		printf("Unable to read port setting with %s\n", BTC9100_CMB_PORTNAME);
		return 0;
	}
	memset(&newPortSetting, 0, sizeof(newPortSetting)); /* Initialize new port setting struct */
	/* Set new port setting struct */
	newPortSetting.c_cflag = BTC9100_CMB_DATABITS | BTC9100_CMB_PARITY | BTC9100_CMB_STOPBITS | CLOCAL | CREAD;/* Setting about control modes*/
	/* Select iflag mode */
	switch (BTC9100_CMB_PARITY) {
	case 0:
		newPortSetting.c_iflag = IGNPAR;/* Ignore check */
		break;
	case PARODD:
	case PARENB:
		newPortSetting.c_iflag = INPCK;/* Enable odd/even check */
		break;
	default:
		printf("Invaild parity with %s.\n", BTC9100_CMB_PORTNAME);
		return 0;
	}
	
	newPortSetting.c_oflag = 0;/* Setting about output modes */
	newPortSetting.c_lflag = 0;/* Setting about local modes */
	newPortSetting.c_cc[VMIN] = 0;/* Defines minimum number of characters for noncanonical read */
	newPortSetting.c_cc[VTIME] = 0;/* Defines timeout in deciseconds for noncanonical read */

	/* Set input baud rate in termios structure */
	if (cfsetispeed(&newPortSetting, BTC9100_CMB_BAUDRATE) == -1) {
		close(comport);
		printf("Unable to set input baud rate with %s\n", BTC9100_CMB_PORTNAME);
		return 0;
	}
	/* Set output baud rate in termios structure */
	if (cfsetospeed(&newPortSetting, BTC9100_CMB_BAUDRATE) == -1) {
		close(comport);
		printf("Unable to set output baud rate with %s\n", BTC9100_CMB_PORTNAME);
		return 0;
	}
	/* Set parameters into comport's termios structure.*/
	if (tcsetattr(comport, TCSANOW, &newPortSetting) == -1) {
		tcsetattr(comport, TCSANOW, &oldPortSetting);/* Restore to old port setting */
		close(comport);
		printf("Unable to set port setting with %s.\n", BTC9100_CMB_PORTNAME);
		return 0;
	}
	/* Get the status of modem bits */
	if (ioctl(comport, TIOCMGET, &status) == -1) {
		tcsetattr(comport, TCSANOW, &oldPortSetting);/* Restore to old port setting */
		printf("Unable to get port setting with %s.\n", BTC9100_CMB_PORTNAME);
		return 0;
	}
	status |= TIOCM_DTR | TIOCM_RTS;/* Configure status data terminal ready and request to send */
	/* Set the status of modem bits */
	if (ioctl(comport, TIOCMSET, &status) == -1) {
		tcsetattr(comport, TCSANOW, &oldPortSetting);/* Restore to old port setting */
		printf("Unable to set port setting with %s.\n", BTC9100_CMB_PORTNAME);
		return 0;
	}
	return 1;/* Success to set connection */
}

//Get temperature from BTC-9100
void tempRead() {
	
}

//Set temperature value, celsius, to BTC-9100
void tempSetup() {

}

//Set fan value, speed, to BTC-9100
void fanSetup() {

}

//Turn on fan
void fanOn() {

}

//Turn off fan
void fanOff(){

}

//Set bulb value, brightness, to BTC-9100
void bulbSetup() {

}

//Turn on bulb
void bulbOn() {

}

//Turn off bulb
void bulbOff() {

}

int main() {
	char args[2][MAX_STR_LENS] = { "TempSetup", "48.7" };//For test
	//// Connection Myconnect = new Connection();
	//// SerialPort comport = new SerialPort();
	//// Modbus.Device.ModbusSerialMaster master = null;
	////comport.Close();

	/* Set information of RS232 connection like port name, parity bits, stop bits, data bits, baudrate and slave ID. */
	if (!setConnectionInfo()) {
		printf("Fail to set connection with chamber.");
		return 0;
	};

	switch (cmdConvertToNum(args[0])) {
	case TEMP_READ:
		tempRead();
		break;
	case TEMP_SETUP:
		tempSetup();
		break;
	case FAN_SETUP:
		fanSetup();
		break;
	case FAN_ON:
		fanOn();
		break;
	case FAN_OFF:
		fanOff();
		break;
	case BULB_ON:
		bulbOn();
		break;
	case BULB_OFF:
		bulbOff();
		break;
	default:
		printf("Error command or command convert failured.\n");
	};
	return 0;
}