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

/* Temperature controller connection infomation */
#define BTC9100_CMB_PORTNAME "/dev/ttyUSB0"
#define BTC9100_MODBUS_PARITY 'E' /* 'N' for none, 'E' for even, 'O' for odd */
#define BTC9100_CMB_STOPBITS 1
#define BTC9100_CMB_DATABITS 8 /* allow value are 5,6,7 and 8 */
#define BTC9100_MODBUS_BAUDRATE 9600
#define BTC9100_MODBUS_SLAVE_ID 2
#define BTC9100_SP1_REG_ADDR 0
#define BTC9100_PV_REG_ADDR 64

/* Controller connection information */
#define EZD305F_CMB_PORTNAME "/dev/ttyUSB0"
#define EZD305F_CMB_PARITY 0   /* 0 = None, PARENB = Even, PARODD = Odd */
#define EZD305F_CMB_STOPBITS 1 /* CSTOPB = 2 bits, otherwise = 1bits */
#define EZD305F_CMB_DATABITS CS8
#define EZD305F_CMB_BAUDRATE B9600 /* B + baudrate */

/* SlaveID with different EZD305F device */
#define FAN_EZD305F_SLAVE_ID  0x0A
#define BULB_EZD305F_SLAVE_ID 0x00

/* Command Define */
#define TEMP_READ 0
#define TEMP_SETUP 1
#define FAN_SETUP 2
#define FAN_ON 3
#define FAN_OFF 4
#define BULB_SETUP 5
#define BULB_ON 6
#define BULB_OFF 7

/* Web socket reference */
#define LISTEN_PORT 1500
#define MAX_LISTEN 5
#define MAX_DATA_LENS 100

/* Static number */
#define TEMP_RANGE 0.5


/* Global variable */
int comport;
int err;
int sockfd;
struct termios portSetting;
modbus_t *modbusport = NULL;
int fanSpeed;
int bulbLight;
float envTemp;
float targetTemp;


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
static unsigned char* percentToHex(int);

/* Convert command of string type to corresponding number */
int cmdConvertToNum(char *str)
{
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

/* Convert 0-100% to 0x00-0xff (0xUV) return {0x0U, 0x0V} */
unsigned char* percentToHex(int value){
    unsigned char *output;
    output = malloc(sizeof(char)*2);
    if (value < 0 || value > 100){
        printf("Input value: [%d] exceed range 0-100\nOr just ckeck, no value wanted to set.\n", value);
        //exit(1)
    }
    else{
        printf("value: %d\n", value);
        output[0] = (value * 255 / 100) >> 4;
        output[1] = (value * 255 / 100) & 0x0f;
        return output;
    }
}

/* Set Information about serial port connection with RS 232, 
 * info about port name, parity bits, stop bits, data bits, baudrate and slave ID. */
int openSerial()
{
    int status; /* Record ioctl message */
    /* All flag define is on /usr/include/asm-generic/fcntl.h */

    /* O_RDWR meaning file will in read&write mode
     * O_NOCTTY meaning file is about terminal device
     * O_NDELAY | O_NONBLOCK meaning file will in nonblocking mode */
    comport =
        open(EZD305F_CMB_PORTNAME,
             O_RDWR | O_NOCTTY | O_SYNC |
                 O_NONBLOCK); /* Open file can read&write in nonblocking mode */
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
    portSetting.c_cflag |=
        (CLOCAL | CREAD); /* Ignore modem control lines and enable receiver. */
    portSetting.c_cflag &= ~EZD305F_CMB_PARITY;
    portSetting.c_cflag &= ~CSTOPB; /* Set one stop bits, if wanted set two stop
                                       bits then &= CSTOPB */
    portSetting.c_cflag &= ~CSIZE;  /* Nonuse with character size mask */
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
    if (tcsetattr(comport, TCSANOW, &portSetting) ==
        -1) { /* the change occurs immediately. */
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
    usleep(10000); /* suspend excution for 0.01 seconds */
    return 1;      /* Success to set connection */
}

int closeSerial()
{
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

int openModbus()
{
    modbusport = modbus_new_rtu(BTC9100_CMB_PORTNAME, BTC9100_MODBUS_BAUDRATE,
                                BTC9100_MODBUS_PARITY, BTC9100_CMB_DATABITS,
                                BTC9100_CMB_STOPBITS);
    if (modbusport == NULL) {
        fprintf(stderr, "Unable to allocate libmodbus context\n");
        return 0;
    }
    // modbus_rtu_set_serial_mode(MODBUS_RTU_RS485); /* Set serial mode with
    // RS485 or RS232 */
    modbus_set_debug(
        modbusport,
        1); /* Verbose messages are displayed on stdout and stderr */
    modbus_set_slave(modbusport, BTC9100_MODBUS_SLAVE_ID);
    if (modbus_connect(modbusport) == -1) {
        fprintf(stderr, "Modbus connection failed:%s\n",
                modbus_strerror(errno));
        return 0;
    }
    return 1;
}

int closeModbus()
{
    /* No return value with below func */
    modbus_close(modbusport);
    modbus_free(modbusport);
    return 1;
}

/* Send device value to controller */
void devSetup(unsigned char slaveID, int value)
{
    unsigned char *valueHex; 
    valueHex = percentToHex(value);
    if(!valueHex){
        unsigned char comm[] = {0xE0, slaveID, 0x00, 0x57, 0x03,
                                0x00, 0x00, valueHex[0], valueHex[1], 0xFE};
        write(comport, comm, sizeof(comm));
    }
    free(valueHex);
}

/* Control device with different slaveID and control signal */
/* ctl_s = 1 mean on, ctl_s = 0 mean off */
void devOnOff(unsigned char slaveID, unsigned char ctl_s)
{
    unsigned char comm[] = {0xE0, slaveID, 0x00, 0x57, 0x01,
                            0x00, 0x00, 0x00, ctl_s, 0xFE};
    write(comport, comm, sizeof(comm));         
}

/* Get temperature from BTC-9100 through modbus */
void tempRead()
{
    uint16_t *tab_reg; /* Define register array */
    tab_reg = malloc(sizeof(uint16_t));

    printf("\n----------------\n");
    /* Read target environment temperature */
    err = modbus_read_registers(modbusport, BTC9100_SP1_REG_ADDR, 1, tab_reg);
    if (err == -1) {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
        return;
    }
    targetTemp = (float) (*tab_reg - 19999) / 10;
    printf("reg[%d] = %f(%d)\n", BTC9100_SP1_REG_ADDR, targetTemp, *tab_reg);

    /* Read current environment temperature */
    err = modbus_read_registers(modbusport, BTC9100_PV_REG_ADDR, 1, tab_reg);
    if (err == -1) {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
        return;
    }
    envTemp = (float) (*tab_reg - 19999) / 10;
    printf("reg[%d] = %f(%d)\n", BTC9100_PV_REG_ADDR, envTemp, *tab_reg);
    usleep(500000); /* Sleep for 0.5s */
    return;
}

/* Set temperature value, celsius, to BTC-9100 */
void tempSetup(float inputTemp)
{
    uint16_t tab_reg; /* Define register array */
    /*tab_reg = malloc(sizeof(uint16_t));*/

    tab_reg = inputTemp * 10 + 19999;
    err = modbus_write_registers(modbusport, BTC9100_SP1_REG_ADDR, 1, &tab_reg);
    if (err == -1) {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
        return;
    }
    return;
}

void openSocket()
{
    struct sockaddr_in localAddr;
    sockfd = socket(AF_INET, SOCK_STREAM, 0); /* IPv4 TCP connection */
    if (sockfd == -1) {
        fprintf(stderr, "Socket build failed:%d\n", errno);
        return;
    }

    localAddr.sin_family = AF_INET;
    localAddr.sin_port = htons(LISTEN_PORT);
    localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    bzero(&(localAddr.sin_zero), 8);
    if (bind(sockfd, (struct sockaddr *) &localAddr, sizeof(struct sockaddr)) <
        0) {
        fprintf(stderr, "Socket bind error\n");
        return;
    }
    return;
}

void listenFromSocket()
{
    int newfd;
    int sin_size;
    struct sockaddr_in serverAddr;
    char buf[MAX_DATA_LENS];
    char returnBuf[MAX_DATA_LENS];

    listen(sockfd, MAX_LISTEN);

    while (1) {
        sin_size = sizeof(struct sockaddr_in);
        newfd = accept(sockfd, (struct sockaddr *) &serverAddr, &sin_size);
        if (newfd == -1) {
            printf("Receive failed\n");
            return;
        } else {
            printf("Receive success\n");
            memset(buf, 0, MAX_DATA_LENS);
            memset(returnBuf, 0, MAX_DATA_LENS);
            recv(newfd, buf, MAX_DATA_LENS,
                 0); /* Waitting for input command by newfd */
            //printf("Receive from web server:%s\n",&buf);//for test
            dealCommand(buf, returnBuf);
            send(newfd, returnBuf, strlen(returnBuf), 0);
            printf("%s %d\n", returnBuf, strlen(returnBuf));
        }
    }
    return;
}

/* Deal with chamber temperature control with thread 1 */
void *tempControl(void *tmp)
{
    int timeCounter = 0;

    while (1) {
        timeCounter++;
        /* Read temperature from chamber every 5 time unit(0.01s) */
        if (timeCounter % 5 == 0) {
            timeCounter = 0;
            /* Call temp read function */
            if (!openModbus()) {
                printf(
                    "Fail to set connection between chamber and PC with "
                    "modbus.");
                return 0;
            };
            tempRead();
            closeModbus();
            /* Chamber too hot, adjust fan fast or light the bulb */
            if (envTemp - targetTemp > TEMP_RANGE && fanSpeed < 100) {
                /* Adjust fan fast */
                fanSpeed += 1;
                if (!openSerial()) {
                    printf(
                        "Fail to set connection between chamber and PC with "
                        "serial port.");
                    return 0;
                };
                devSetup(FAN_EZD305F_SLAVE_ID,fanSpeed);
                closeSerial();
            }
            /* Chamber too cold, adjust fan slow or dim the bulb */
            else if (targetTemp - envTemp > TEMP_RANGE && fanSpeed > 0) {
                /* Adjust fan slow */
                fanSpeed -= 1;
                if (!openSerial()) {
                    printf(
                        "Fail to set connection between chamber and PC with "
                        "serial port.");
                    return 0;
                };
                devSetup(FAN_EZD305F_SLAVE_ID,fanSpeed);
                closeSerial();
            }
        }
        usleep(10000); /* suspend excution for 0.01 seconds */
    }
    return 0;
}

/* Do correspondence action with command from web server */
void dealCommand(char *buf, char *returnBuf)
{
    char *p;
    if (strstr(buf, "TempRead") != NULL) {
        /* Command format: "TempRead"*/
        snprintf(returnBuf, MAX_DATA_LENS, "envTemp:%f, targetTemp:%f\n",
                 envTemp, targetTemp);
        return;
    } else if (strstr(buf, "TempSetup") != NULL) {
        /* Command format: "TempSetup 40.5"*/
        p = strtok(buf, " ");
        p = strtok(NULL, "");
        targetTemp = atof(p);
        if (!openModbus()) {
            printf(
                "Fail to set connection between chamber and PC with modbus.");
            return;
        };
        tempSetup(targetTemp);
        closeModbus();
        snprintf(returnBuf, MAX_DATA_LENS, "OK, Now targetTemp is %.2f\n",
                 targetTemp);
        return;
    } else if (strstr(buf, "FanSetup") != NULL) {
        /* Command format: "FanSetup 50"*/
        p = strtok(buf, " ");
        p = strtok(NULL, "");
        fanSpeed = atof(p);
        if (!openSerial()) {
            printf(
                "Fail to set connection between chamber and PC with serial "
                "port.");
            return;
        };
        devSetup(FAN_EZD305F_SLAVE_ID,fanSpeed);
        closeSerial();
        snprintf(returnBuf, MAX_DATA_LENS, "OK, Now fan speed is %d\n",
                 fanSpeed);
        return;
    } else if (strstr(buf, "FanOn") != NULL) {
        /* Command format: "FanOn" */
        if (!openSerial()) {
            printf(
                "Fail to set connection between chamber and PC with serial "
                "port.");
            return;
        };
        devOnOff(FAN_EZD305F_SLAVE_ID, 1);
        closeSerial();
        snprintf(returnBuf, MAX_DATA_LENS, "OK, Fan On\n");

        return;
    } else if (strstr(buf, "FanOff") != NULL) {
        /* Command format: "FanOff" */
        if (!openSerial()) {
            printf(
                "Fail to set connection between chamber and PC with serial "
                "port.");
            return;
        };
        devOnOff(FAN_EZD305F_SLAVE_ID, 0);
        closeSerial();
        snprintf(returnBuf, MAX_DATA_LENS, "OK, Fan Off\n");
    } else if (strstr(buf, "BulbOn") != NULL) {
        /* Command format: "BulbOn" */
        if (!openSerial()) {
            printf(
                "Fail to set connection between chamber and PC with serial "
                "port.");
            return;
        };
        devOnOff(BULB_EZD305F_SLAVE_ID, 1);
        closeSerial();
        snprintf(returnBuf, MAX_DATA_LENS, "OK, Bulb On\n");                
    } else if (strstr(buf, "BulbOff") != NULL) {
        /* Command format: "BulbOn" */
        if (!openSerial()) {
            printf(
                "Fail to set connection between chamber and PC with serial "
                "port.");
            return;
        };
        devOnOff(BULB_EZD305F_SLAVE_ID, 0);
        closeSerial();
        snprintf(returnBuf, MAX_DATA_LENS, "OK, Bulb Off\n");                 
    } else {
        printf("***Error command or command convert failured.***\n");
    }
}

int main(int argc, char *argv[])
{
    // char args[2][50] = {"FanOn", "48.7"};  // For test
    if (argc < 2){
        pthread_t th1;
        pthread_create(&th1, NULL, tempControl, "Child");
        usleep(10000); /* suspend excution for 0.01 seconds */
        openSocket();
        listenFromSocket();
        pthread_join(th1, NULL);
    } else {
        // interface for test
        switch (cmdConvertToNum(argv[1])) {
        case TEMP_READ:
            if (!openModbus()) return 0;
            tempRead();
            closeModbus();
            break;
        case TEMP_SETUP:
            if (!openModbus()) return 0;
            tempSetup(atof(argv[2]));
            closeModbus();
            break;
        case FAN_SETUP:
            if (!openSerial()) return 0;
            devSetup(FAN_EZD305F_SLAVE_ID,atoi(argv[2]));
            closeSerial();
            break;
        case FAN_ON:
            if (!openSerial()) return 0;
            devOnOff(FAN_EZD305F_SLAVE_ID, 1);
            closeSerial();
            break;
        case FAN_OFF:
            if (!openSerial()) return 0;
            devOnOff(FAN_EZD305F_SLAVE_ID, 0);
            closeSerial();
            break;
        case BULB_ON:
            if (!openSerial()) return 0;
            devOnOff(BULB_EZD305F_SLAVE_ID, 1);
            closeSerial();
            break;
        case BULB_OFF:
            if (!openSerial()) return 0;
            devOnOff(BULB_EZD305F_SLAVE_ID, 0);
            closeSerial();
            break;
        default:
            printf("Error command or command convert failured.\n");
        };
    }

    return 0;
}
