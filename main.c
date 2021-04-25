#include <fcntl.h>
#include <modbus.h>
#include <modbus/modbus.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>

/* Temperature controller connection infomation */
#define BTC9100_CMB_PORTNAME    "/dev/ttyUSB0"
#define BTC9100_MODBUS_PARITY   'E' /* 'N' for none, 'E' for even, 'O' for odd */
#define BTC9100_CMB_STOPBITS    1
#define BTC9100_CMB_DATABITS    8   /* allow value are 5,6,7 and 8 */
#define BTC9100_MODBUS_BAUDRATE 9600
#define BTC9100_MODBUS_SLAVE_ID 2
#define BTC9100_SP1_REG_ADDR    0
#define BTC9100_PV_REG_ADDR     64

/* Fan controller connection information */
#define EZD305F_CMB_PORTNAME    "/dev/ttyUSB0"
#define EZD305F_CMB_PARITY      0     /* 0 = None, PARENB = Even, PARODD = Odd */
#define EZD305F_CMB_STOPBITS    1     /* CSTOPB = 2 bits, otherwise = 1bits */
#define EZD305F_CMB_DATABITS    CS8
#define EZD305F_CMB_BAUDRATE    B9600 /* B + baudrate */
#define EZD305F_SLAVE_ID        "0A"

/* Command Define */
#define TEMP_READ   0
#define TEMP_SETUP  1
#define FAN_SETUP   2
#define FAN_ON      3
#define FAN_OFF     4
#define BULB_SETUP  5
#define BULB_ON     6
#define BULB_OFF    7

/* Global variable */
int comport;  
struct termios portSetting;
modbus_t *modbusport = NULL;
float envTemp;
float targetTemp;
int err;

/* Function */
static int cmdConvertToNum(char*);
static void tempRead();
static void tempSetup(float);
static void fanSetup();
static void fanOn();
static void fanOff();
static void bulbSetup();
static void bulbOn();
static void bulbOff();
static int  openSerial();
static int  closeSerial();
static int  openModbus();
static int  closeModbus();

/* Convert command of string type to corresponding number */
int cmdConvertToNum(char* str) {
	printf("Now Command:%s\n", str);
    if (!strcmp(str, "TempRead"))
        return TEMP_READ;
    else if (!strcmp(str, "TempSetup"))
        return TEMP_SETUP;
    else if (!strcmp(str, "FanSetup"))
        return FAN_SETUP;
    else if (!strcmp(str, "FanOn"))
        return FAN_ON;
    else if (!strcmp(str, "FanOff"))
        return FAN_OFF;
    else if (!strcmp(str, "BulbOn"))
        return BULB_ON;
    else if (!strcmp(str, "BulbOff"))
        return BULB_OFF;
}

/* Set Information about serial port connection */
int openSerial() {
    int status; /* Record ioctl message */
    /* All flag define is on /usr/include/asm-generic/fcntl.h */

    /* O_RDWR meaning file will in read&write mode
     * O_NOCTTY meaning file is about terminal device
     * O_NDELAY | O_NONBLOCK meaning file will in nonblocking mode */
    comport =
        open(EZD305F_CMB_PORTNAME,
             O_RDWR | O_NOCTTY |
                 O_NDELAY | O_NONBLOCK); /* Open file can read&write in nonblocking mode */
    if (comport == -1) {
        printf("Unable to open comport with %s\n", EZD305F_CMB_PORTNAME);
        return 0;
    }

    /* Get parameters associated with the object referred by fd and stores them
     * in the termios structure.*/
    if (tcgetattr(comport, &portSetting) == -1) {
        close(comport);
        printf("Unable to read port setting with %s\n", EZD305F_CMB_PORTNAME);
        return 0;
    }

    /* Set new port setting struct */
    portSetting.c_cflag |= (CLOCAL | CREAD); /* Ignore modem control lines and enable receiver. */ 
    portSetting.c_cflag &= ~EZD305F_CMB_PARITY;
    portSetting.c_cflag &= ~CSTOPB; /* Set one stop bits, if wanted set two stop bits then &= CSTOPB */
    portSetting.c_cflag &= ~CSIZE; /* Nonuse with character size mask */
    portSetting.c_cflag |= EZD305F_CMB_DATABITS;
    /* Disable canonical mode(always wtih set ECHOE), echo input characters, 
     * When any of the characters INTR, QUIT, SUSP, or DSUSP are received, 
     * generate the corresponding signal. */
    portSetting.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
    
    portSetting.c_cc[VMIN] =
        0; /* Defines minimum number of characters for noncanonical read */
    portSetting.c_cc[VTIME] =
       100; /* Defines timeout in deciseconds for noncanonical read */

    /* Set input baud rate in termios structure */
    if (cfsetispeed(&portSetting, EZD305F_CMB_BAUDRATE) == -1) {
        close(comport);
        printf("Unable to set input baud rate with %s\n", EZD305F_CMB_PORTNAME);
        return 0;
    }
    /* Set output baud rate in termios structure */
    if (cfsetospeed(&portSetting, EZD305F_CMB_BAUDRATE) == -1) {
        close(comport);
        printf("Unable to set output baud rate with %s\n",
               EZD305F_CMB_PORTNAME);
        return 0;
    }
    /* Set parameters into comport's termios structure.*/
    if (tcsetattr(comport, TCSANOW, &portSetting) == -1) { /* the change occurs immediately. */
        close(comport);
        printf("Unable to set port setting with %s.\n", EZD305F_CMB_PORTNAME);
        return 0;
    }
    /* Get the status of modem bits */
    if (ioctl(comport, TIOCMGET, &status) == -1) {
        printf("Unable to get port setting with %s.\n", EZD305F_CMB_PORTNAME);
        return 0;
    }
    status |= TIOCM_DTR | TIOCM_RTS; /* Configure status data terminal ready and
                                        request to send */
    /* Set the status of modem bits */
    if (ioctl(comport, TIOCMSET, &status) == -1) {
        printf("Unable to set port setting with %s.\n", EZD305F_CMB_PORTNAME);
        return 0;
    }
    usleep(10000);/* suspend excution for 0.01 seconds */
    return 1; /* Success to set connection */
}

int closeSerial() {
    int status; /* Record ioctl message */

    if (ioctl(comport, TIOCMGET, &status) == -1)
        printf("Unable to get port status when closing connection.\n");
    status &= ~(TIOCM_DTR | TIOCM_RTS); /* Turn off DTR and RTS */
    if (ioctl(comport, TIOCMSET, &status) == -1)
        printf("Unable to set port status when closing connection.\n");
    tcsetattr(comport, TCSANOW, &portSetting);
    close(comport);
    return 1;
}

int openModbus(){
    modbusport = modbus_new_rtu(BTC9100_CMB_PORTNAME, BTC9100_MODBUS_BAUDRATE, BTC9100_MODBUS_PARITY, BTC9100_CMB_DATABITS, BTC9100_CMB_STOPBITS);
    if(modbusport == NULL){
        fprintf(stderr, "Unable to allocate libmodbus context\n");
        return 0;
    }
    // modbus_rtu_set_serial_mode(MODBUS_RTU_RS485); /* Set serial mode with RS485 or RS232 */
    modbus_set_debug(modbusport, 1);/* Verbose messages are displayed on stdout and stderr */
    modbus_set_slave(modbusport, BTC9100_MODBUS_SLAVE_ID);
    if (modbus_connect(modbusport) == -1){
        fprintf(stderr, "Modbus connection failed:%s\n", modbus_strerror(errno));
        return 0;
    }
    return 1;
}

int closeModbus(){
    /* No return value with below func */
    modbus_close(modbusport);
    modbus_free(modbusport);
    return 1;
}

/* Set speed to fan controller */
void fanSetup() {
    //unsigned char comm[] = {0xE0, 0x0A, 0x00, 0x57, 0x03,
    //                        0x00, 0x00, 0x, 0x, 0xFE};
}

void fanOn() {
    unsigned char comm[] = {0xE0, 0x0A, 0x00, 0x57, 0x01,
                            0x00, 0x00, 0x00, 0x01, 0xFE};
    write(comport, comm, sizeof(comm));
    printf("Fan On\n");
}

void fanOff() {
    unsigned char comm[] = {0xE0, 0x0A, 0x00, 0x57, 0x01,
                            0x00, 0x00, 0x00, 0x00, 0xFE};
    write(comport, comm, sizeof(comm));
    printf("Fan Off\n");
}

/* Get temperature from BTC-9100 through modbus */
void tempRead() {
    uint16_t *tab_reg;  /* Define register array */
    tab_reg = malloc(sizeof(uint16_t)); 

    printf("\n----------------\n");
    /* Read current environment temperature */
    err = modbus_read_registers(modbusport, BTC9100_SP1_REG_ADDR, 1, tab_reg);
    if (err == -1) {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
        return;
    }
    envTemp = (float)(*tab_reg-19999)/10;
    printf("reg[%d] = %f(%d)\n", BTC9100_SP1_REG_ADDR, envTemp, *tab_reg);

    /* Read target environment temperature */
    err = modbus_read_registers(modbusport, BTC9100_PV_REG_ADDR, 1, tab_reg);
    if (err == -1) {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
        return;
    }
    targetTemp = (float)(*tab_reg-19999)/10;
    printf("reg[%d] = %f(%d)\n", BTC9100_PV_REG_ADDR, targetTemp, *tab_reg);
    usleep(500000); /* Sleep for 0.5s */
    return;
}

/* Set temperature value, celsius, to BTC-9100 */
void tempSetup(float inputTemp) {
    uint16_t *tab_reg;  /* Define register array */
    tab_reg = malloc(sizeof(uint16_t)); 

    tab_reg[BTC9100_PV_REG_ADDR] = inputTemp*10 + 19999;
    err = modbus_write_registers(modbusport, BTC9100_PV_REG_ADDR, 1, tab_reg);
    if(err == -1){
        fprintf(stderr, "%s\n", modbus_strerror(errno));
        return;
    }
    return;
}

/* Set bulb value, brightness, to BTC-9100 */
void bulbSetup() {}

/* Turn on bulb */
void bulbOn() {}

/* Turn off bulb */
void bulbOff() {}

int main(int argc, char* argv[]) {
    //char args[2][50] = {"FanOn", "48.7"};  // For test
    
    switch (cmdConvertToNum(argv[1])) {
        case TEMP_READ:
            if (!openModbus()) {
                printf("Fail to set connection between chamber and PC with modbus.");
                return 0;
            };
            tempRead();
            closeModbus();
            break;
        case TEMP_SETUP:
            if (!openModbus()) {
                printf("Fail to set connection between chamber and PC with modbus.");
                return 0;
            };
            tempSetup(atof(argv[2]));
            closeModbus();
            break;
        case FAN_SETUP:
            /* Set information of RS232 connection like port name, parity bits, stop
            * bits, data bits, baudrate and slave ID. */
            if (!openSerial()) {
                printf("Fail to set connection between chamber and PC with serial port.");
                return 0;
            };
            fanSetup(argv[2]);
            closeSerial();
            break;
        case FAN_ON:
            /* Set information of RS232 connection like port name, parity bits, stop
            * bits, data bits, baudrate and slave ID. */
            if (!openSerial()) {
                printf("Fail to set connection between chamber and PC with serial port.");
                return 0;
            };
            fanOn();
            closeSerial();
            break;
        case FAN_OFF:
            /* Set information of RS232 connection like port name, parity bits, stop
            * bits, data bits, baudrate and slave ID. */
            if (!openSerial()) {
                printf("Fail to set connection between chamber and PC with serial port.");
                return 0;
            };
            fanOff();
            closeSerial();
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
