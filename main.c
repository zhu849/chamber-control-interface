/* Implement core function of MCU interface */
#include "main.h"

/* Global variable */
int comport;
int err;
int sockfd;
struct termios portSetting;
modbus_t *modbusport = NULL;
pthread_t th1, th2;
/* Chamber parameter */
float fanSpeed = 0;
float bulbLight = 0;
int fanSwitch = 0;
int bulbSwitch = 0;
float envTemp;
float targetTemp = -1;
int mode = 1;
int mutex = 0;

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
    else if (!strcmp(str, "BulbSetup"))
        return BULB_SETUP;
    else if (!strcmp(str, "FanOn"))
        return FAN_ON;
    else if (!strcmp(str, "FanOff"))
        return FAN_OFF;
    else if (!strcmp(str, "BulbOn"))
        return BULB_ON;
    else if (!strcmp(str, "BulbOff"))
        return BULB_OFF;
    else if (!strcmp(str, "ChangeMode"))
        return CHANGE_MODE;
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
        //printf("value: %d\n", value);
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
    portSetting.c_cflag |= (CLOCAL | CREAD); /* Ignore modem control lines and enable receiver. */
    portSetting.c_cflag &= ~EZD305F_CMB_PARITY;
    portSetting.c_cflag &= ~CSTOPB; /* Set one stop bits, if wanted set two stop bits then &= CSTOPB */
    portSetting.c_cflag &= ~CSIZE;  /* Nonuse with character size mask */
    portSetting.c_cflag |= EZD305F_CMB_DATABITS;

    /* Disable canonical mode(always wtih set ECHOE), echo input characters,
     * When any of the characters INTR, QUIT, SUSP, or DSUSP are received,
     * generate the corresponding signal. */
    portSetting.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    portSetting.c_cc[VMIN] = 0; /* Defines minimum number of characters for noncanonical read */
    portSetting.c_cc[VTIME] = 100; /* Defines timeout in deciseconds for noncanonical read */

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
        0); /* Verbose messages are displayed on stdout and stderr */

    usleep(10000); /* suspend excution for 0.01 seconds */

    modbus_set_slave(modbusport, BTC9100_MODBUS_SLAVE_ID);
    
    // set timeout
    modbus_set_response_timeout(modbusport, 1, 500000);

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
void devSetup(unsigned char slaveID, float f_value)
{
    int value = (int)f_value;
    unsigned char *valueHex; 
    valueHex = percentToHex(value);
    if(valueHex[0]<0x10 && valueHex[1]<0x10){
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

    /*
    // Read target environment temperature 
    err = modbus_read_registers(modbusport, BTC9100_SP1_REG_ADDR, 1, tab_reg);
    if (err == -1) {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
        return;
    }
    targetTemp = (float) (*tab_reg - 19999) / 10;
    printf("reg[%d] = %f(%d)\n", BTC9100_SP1_REG_ADDR, targetTemp, *tab_reg);
    */

    /* Read current environment temperature */
    err = modbus_read_registers(modbusport, BTC9100_PV_REG_ADDR, 1, tab_reg);
    if (err == -1) {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
        return;
    }
    envTemp = (float) (*tab_reg - 19999) / 10;
    //printf("reg[%d] = %f(%d)\n", BTC9100_PV_REG_ADDR, envTemp, *tab_reg);
    usleep(50000); /* Sleep for 0.05s */
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
    sin_size = sizeof(struct sockaddr_in);

    while(1){
        newfd = accept(sockfd, (struct sockaddr *) &serverAddr, &sin_size);
        printf("New connection is accept from webserver!\n");

        while (1) {
            if (newfd == -1){
                printf("Receive connection from client is failed\n");
                break;
            }
            else {
                memset(buf, 0, MAX_DATA_LENS);
                memset(returnBuf, 0, MAX_DATA_LENS);
                /* Waitting for input command by now newfd */
                recv(newfd, buf, MAX_DATA_LENS, 0); 
                printf("Recv msg from webserver, command: %s\n",&buf);
                dealCommand(buf, returnBuf);
                send(newfd, returnBuf, strlen(returnBuf), 0);
            }
        }
        printf("One connection is close from webserver!\n");
    }
    return;
}

/* Deal with chamber temperature control with thread-1 */
void *tempControl(void *tmp)
{
    char c;

    while (1) {
        /* Just work on auto mode */
        if(mode == AUTO){

            while(mutex) 
                usleep(1);
            mutex = 1;

            if (!openModbus()) {
                printf(
                    "Fail to set connection between chamber and PC with "
                    "modbus.");
                return 0;
            };
            usleep(20000);
            tempRead();
            usleep(20000);
            closeModbus();

            mutex = 0;

            // if target temperature is -1 mean none start
            if(targetTemp != -1){
                /* Chamber is ideal status */
                if(envTemp - targetTemp <= NORMAL_RANGE && targetTemp - envTemp <= NORMAL_RANGE){
                    printf("Chamber is in target environment!\n");
                }
                /* Chamber is very hot */
                else if (envTemp - targetTemp > VERY_HOT_THR) {
                    bulbLight -= 10;
                    fanSpeed = 100;
                    fanSwitch = 1;
                    bulbSwitch = 0;
                    bulbLight = 0;
                }
                /* Chamber is a little hot */
                else if(envTemp - targetTemp > HOT_THR){
                    bulbLight -= 5;
                    fanSpeed = 100;
                    fanSwitch = 1;
                    bulbSwitch = 0;
                    bulbLight = 0;
                }
                /* Chamber is a little cold*/
                else if(targetTemp - envTemp > COLD_THR){
                    bulbLight += 3;
                    fanSpeed = 0;
                    fanSwitch = 0;
                    bulbSwitch = 1;
                }
                /* Chamber is very cold*/
                else if(targetTemp - envTemp > VERY_COLD_THR){
                    bulbLight += 5;
                    fanSpeed = 0;
                    fanSwitch = 0;
                    bulbSwitch = 1;
                }

                // Boundary check
                if(bulbLight>100)
                    bulbLight = 100;
                else if(bulbLight < 0)
                    bulbLight = 0;

                // Set new configuration about fan
                while(mutex)
                    usleep(1);
                mutex = 1;

                if (!openSerial()) {
                    printf(
                        "Fail to set connection between chamber and PC with "
                        "serial port.");
                    return 0;
                };
                usleep(10000);
                for(int i=0;i<5;i++){ 
                    devOnOff(FAN_EZD305F_SLAVE_ID, fanSwitch);
                    usleep(100);
                }
                usleep(10000);
                for(int i=0;i<5;i++){
                    devSetup(FAN_EZD305F_SLAVE_ID, fanSpeed);
                    usleep(100);
                }
                usleep(10000);
                closeSerial();

                mutex = 0;
                usleep(10000);

                // Set new configuration about bulb
                while(mutex)
                    usleep(1);
                mutex = 1;

                    if (!openSerial()) {
                        printf(
                            "Fail to set connection between chamber and PC with "
                            "serial port.");
                        return 0;
                    };           
                    usleep(10000);
                    for(int i=0;i<5;i++){
                        devOnOff(BULB_EZD305F_SLAVE_ID, bulbSwitch);
                        usleep(100);
                    }
                    usleep(10000);
                    for(int i=0;i<5;i++){
                        if(bulbLight >= 15)
                            devSetup(BULB_EZD305F_SLAVE_ID, bulbLight);
                        usleep(100);
                    }
                    usleep(10000);
                    closeSerial();

                mutex = 0;
                usleep(10000);
            }
        
            printf("AUTO - Fan switch:%d, Bulb switch:%d, Bulb Light:%.2f, Env Temp:%.2f, Target Temp:%.2f\n\n", fanSwitch, bulbSwitch, bulbLight, envTemp, targetTemp);
        }
        printf("now mode:%d\n",mode);
        usleep(500000); /* suspend excution for 0.5 seconds */
    }

    return 0;
}

/* Do correspondence action with command from web server */
void dealCommand(char *buf, char *returnBuf)
{
    char *p;

    if(strstr(buf, "ChangeMode") != NULL){
        /* Command format: "ChangeMode 1"*/
        p = strtok(buf, " ");
        p = strtok(NULL, "");
        mode = atoi(p);
        printf("now mode is change to %d\n",mode);
        snprintf(returnBuf, MAX_DATA_LENS, "now mode is change to %d\n", mode);
        return;
    } else if(strstr(buf, "DataRead") != NULL){
        /* Command format: "DataRead"*/
        snprintf(returnBuf, MAX_DATA_LENS, "envTemp:%f, fanSpeed:%f, bulbLight:%f, targetTemp:%f\n", envTemp, fanSpeed, bulbLight, targetTemp);
        return;
    } else if (strstr(buf, "TempSetup") != NULL) {
        /* Command format: "TempSetup 40.5"*/
        p = strtok(buf, " ");
        p = strtok(NULL, "");
        targetTemp = atof(p);

        while(mutex)
            usleep(1);
        mutex = 1;

        if (!openModbus()) {
            printf(
                "Fail to set connection between chamber and PC with modbus.");
            return;
        };
        usleep(10000);
        tempSetup(targetTemp);
        usleep(10000);
        closeModbus();
        snprintf(returnBuf, MAX_DATA_LENS, "OK, Now targetTemp is %.2f\n",
                 targetTemp);

        mutex = 0;

        return;
    } else if (strstr(buf, "ConfigFileRead") != NULL) {
            /* Command format: "ConfigFileRead" */
            snprintf(returnBuf, MAX_DATA_LENS, "BTC9100_CMB_PORTNAME:%s, BTC9100_MODBUS_PARITY:%c, BTC9100_CMB_STOPBITS:%d, BTC9100_CMB_DATABITS:%d, BTC9100_MODBUS_BAUDRATE:%d, BTC9100_MODBUS_SLAVE_ID:%d, EZD305F_CMB_PORTNAME:%s, EZD305F_CMB_PARITY:%d, EZD305F_CMB_STOPBITS:%d, EZD305F_CMB_DATABITS:%s, EZD305F_CMB_BAUDRATE:%s, FAN_EZD305F_SLAVE_ID:%s, BULB_EZD305F_SLAVE_ID:%d\n", BTC9100_CMB_PORTNAME, BTC9100_MODBUS_PARITY, BTC9100_CMB_STOPBITS, BTC9100_CMB_DATABITS, BTC9100_MODBUS_BAUDRATE, BTC9100_MODBUS_SLAVE_ID, EZD305F_CMB_PARITY,EZD305F_CMB_PORTNAME, EZD305F_CMB_PARITY, EZD305F_CMB_STOPBITS, EZD305F_CMB_DATABITS, EZD305F_CMB_BAUDRATE, FAN_EZD305F_SLAVE_ID, BULB_EZD305F_SLAVE_ID);
            return;
    } 

    /* Just use on manual mode */
    if(mode == MANUAL){
        char *p;
        if (strstr(buf, "FanSetup") != NULL) {
            /* Command format: "FanSetup 50"*/
            p = strtok(buf, " ");
            p = strtok(NULL, "");

            fanSpeed = atof(p);

            while(mutex){}
            mutex = 1;

            if (!openSerial()) {
                printf(
                    "Fail to set connection between chamber and PC with serial "
                    "port.");
                return;
            };
            usleep(10000);
            devSetup(FAN_EZD305F_SLAVE_ID, fanSpeed);
            usleep(10000);
            closeSerial();
            mutex = 0;

            snprintf(returnBuf, MAX_DATA_LENS, "OK, Now fan speed is %d\n",
                     fanSpeed);
            return;
        } else if (strstr(buf, "BulbSetup") != NULL){
            /* Command format: "BulbSetup 50"*/
            p = strtok(buf, " ");
            p = strtok(NULL, "");
            bulbLight = atof(p);

            while(mutex)
                usleep(1);
            mutex = 1;

            if (!openSerial()) {
                printf(
                    "Fail to set connection between chamber and PC with serial "
                    "port.");
                return;
            };
            usleep(10000);
            devSetup(BULB_EZD305F_SLAVE_ID,bulbLight);
            usleep(10000);
            closeSerial();

            mutex = 0;

            snprintf(returnBuf, MAX_DATA_LENS, "OK, Now bulb light is %d\n",
                     bulbLight);
            return;
        } else if (strstr(buf, "FanOn") != NULL) {
            /* Command format: "FanOn" */
            while(mutex)
                usleep(1);
            mutex = 1;

            if (!openSerial()) {
                printf(
                    "Fail to set connection between chamber and PC with serial "
                    "port.");
                return;
            };
            usleep(10000);
            devOnOff(FAN_EZD305F_SLAVE_ID, 1);
            usleep(10000);
            closeSerial();

            mutex = 0;

            snprintf(returnBuf, MAX_DATA_LENS, "OK, Fan On\n");
            return;
        } else if (strstr(buf, "FanOff") != NULL) {
            /* Command format: "FanOff" */
            while(mutex)
                usleep(1);
            mutex = 1;

            if (!openSerial()) {
                printf(
                    "Fail to set connection between chamber and PC with serial "
                    "port.");
                return;
            };
            usleep(10000);
            devOnOff(FAN_EZD305F_SLAVE_ID, 0);
            usleep(10000);
            closeSerial();

            mutex = 0;
            snprintf(returnBuf, MAX_DATA_LENS, "OK, Fan Off\n");

            return;
        } else if (strstr(buf, "BulbOn") != NULL) {
            /* Command format: "BulbOn" */
            while(mutex)
                usleep(1);
            mutex = 1;

            if (!openSerial()) {
                printf(
                    "Fail to set connection between chamber and PC with serial "
                    "port.");
                return;
            };

            usleep(10000);
            devOnOff(BULB_EZD305F_SLAVE_ID, 1);
            usleep(10000);
            closeSerial();

            mutex = 0;
            snprintf(returnBuf, MAX_DATA_LENS, "OK, Bulb On\n");

            return;                
        } else if (strstr(buf, "BulbOff") != NULL) {
            /* Command format: "BulbOn" */
            while(mutex)
                usleep(1);
            mutex = 1;

            if (!openSerial()) {
                printf(
                    "Fail to set connection between chamber and PC with serial "
                    "port.");
                return;
            };
            usleep(10000);
            devOnOff(BULB_EZD305F_SLAVE_ID, 0);
            usleep(10000);
            closeSerial();

            mutex = 0;

            snprintf(returnBuf, MAX_DATA_LENS, "OK, Bulb Off\n");
            return;                 
        } else {
            printf("***Error command or command convert failured.***\n");
        }
    }
}

void sighandler(int signum){
    printf("Interrupt exit, good bye~~\n");
    pthread_exit(&th1);
    usleep(100000);
    closeModbus();
    usleep(100000);
    closeSerial();
    usleep(100000);    
    exit(0);
}

int main(int argc, char *argv[])
{
    // Interrupt condition
    signal(SIGINT, sighandler);

    if(argc < 2){
        // Control thread
        pthread_create(&th1, NULL, tempControl, "Child");
        usleep(10000); /* suspend excution for 0.01 seconds */
        openSocket();
        listenFromSocket();
        pthread_join(th1, NULL);
    }
    else{
        // Interface for function test
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
        case BULB_SETUP:
            if (!openSerial()) return 0;
            devSetup(BULB_EZD305F_SLAVE_ID,atoi(argv[2]));
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
