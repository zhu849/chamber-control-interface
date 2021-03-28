#include <stdio.h>
#include <string.h>

/*Temperature controller RS232 connection infomation*/
#define BTC9100_CMB_PORTNAME	"COM6"
#define BTC9100_CMB_PARITY		"Even"
#define BTC9100_CMB_STOPBITS	1
#define BTC9100_CMB_DATABITS	8
#define BTC9100_CMB_BAUDRATE	9600
#define BTC9100_SLAVE_ID		"2"

/*Fan controller RS232 connection information*/
#define EZD305F_CMB_PORTNAME	"COM6"
#define EZD305F_CMB_PARITY		"None"
#define EZD305F_CMB_STOPBITS	1
#define EZD305F_CMB_DATABITS	8
#define EZD305F_CMB_BAUDRATE	9600
#define EZD305F_SLAVE_ID		"0A"

/*Static Number*/
#define MAX_STR_LENS 50

/*Command Define*/
#define TEMP_READ	0
#define TEMP_SETUP	1
#define FAN_SETUP	2
#define FAN_ON		3
#define FAN_OFF		4
#define BULB_SETUP	5
#define BULB_ON		6
#define BULB_OFF	7

/*Global variable*/

/*Function*/
static int cmdConvertToNum(char*);
static void tempSetup();
static void fanSetup();
static void fanOn();
static void fanOff();
static void bulbSetup();
static void bulbOn();
static void bulbOff();
static int setConnectionInfo();

int main() {
	char args[2][MAX_STR_LENS] = { "TempSetup", "48.7" };//For test
	//// Connection Myconnect = new Connection();
	//// SerialPort comport = new SerialPort();
	//// Modbus.Device.ModbusSerialMaster master = null;
	
	////comport.Close();

	//Set information of RS232 connection like port name, parity bits, stop bits, data bits, baudrate and slave ID.
	if (!setConnectionInfo()) {
		printf("Fail to set connection with chamber.");
		return 0;
	};
	
	switch(cmdConvertToNum(args[0])){
		case TEMP_READ:
			tempReadFromSDS();
			break;
		case TEMP_SETUP:
			break;
		case FAN_SETUP:
			break;
		case FAN_ON:
			break;
		case FAN_OFF:
			break;
		case BULB_ON:
			break;
		case BULB_OFF:
			break;
		default:
			printf("Error command or command convert failured.\n");
	};
	return 0;
}

//Convert command of string type to corresponding number  
int cmdConvertToNum(char *str) {
	if (strcmp(str, "TempRead")) return TEMP_READ;
	else if(strcmp(str, "TempSetup")) return TEMP_SETUP;
	else if (strcmp(str, "FanSetup")) return FAN_SETUP;
	else if(strcmp(str, "FanOn")) return FAN_ON;
	else if (strcmp(str, "FanOff")) return FAN_OFF;
	else if (strcmp(str, "BulbOn")) return BULB_ON;
	else if (strcmp(str, "BulbOff")) return BULB_OFF;
}

//Set Information about serial port connection
int setConnectionInfo() {

	return 0;//Fail to set connection
}

//Get temperature from BTC-9100
void tempReadFromSDS() {
	
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