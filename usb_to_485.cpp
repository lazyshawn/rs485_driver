#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include "RMD_485_Driver.h"
#include <vector>
#include <climits>
#define BAUDRATE B115200
#define DEVICE "/dev/ttyUSB0"


int nFd = 0;
struct termios stNew;
struct termios stOld;

 

//Open Port & Set Port

int SerialInit()

{
    nFd = open(DEVICE, O_RDWR|O_NOCTTY|O_NDELAY);
    if(-1 == nFd)
    {
        perror("Open Serial Port Error!\n");
        return -1;
    }

    if( (fcntl(nFd, F_SETFL, 0)) < 0 )
    {
        perror("Fcntl F_SETFL Error!\n");
        return -1;
    }

    if(tcgetattr(nFd, &stOld) != 0)
    {
        perror("tcgetattr error!\n");
        return -1;
    }

    stNew = stOld;
    cfmakeraw(&stNew);//将终端设置为原始模式，该模式下全部的输入数据以字节为单位被处理
    //set speed
    cfsetispeed(&stNew, BAUDRATE);//115200
    cfsetospeed(&stNew, BAUDRATE);

    //set databits
    stNew.c_cflag |= (CLOCAL|CREAD);
    stNew.c_cflag &= ~CSIZE;
    stNew.c_cflag |= CS8;

    //set parity
    stNew.c_cflag &= ~PARENB;
    stNew.c_iflag &= ~INPCK; 

    //set stopbits
    stNew.c_cflag &= ~CSTOPB;
    stNew.c_cc[VTIME]=0;    //指定所要读取字符的最小数量
    stNew.c_cc[VMIN]=1; //指定读取第一个字符的等待时间，时间的单位为n*100ms
                //假设设置VTIME=0，则无字符输入时read（）操作无限期的堵塞
    tcflush(nFd,TCIFLUSH);  //清空终端未完毕的输入/输出请求及数据。



if( tcsetattr(nFd,TCSANOW,&stNew) != 0 )
    {
        perror("tcsetattr Error!\n");
        return -1;
    }
    return nFd;
}

uint8_t Checksumcrc(uint8_t *aData, uint8_t StartIndex, uint8_t DataLength)
{
  uint8_t crc = 0;
  uint8_t i = 0;
  for(i=StartIndex; i<(StartIndex + DataLength); i++)
  {
   crc +=  aData[i] ;	  
  }	
  return crc;
}
 

int main(int argc, char **argv)

{   
    RMD_485_Driver RMD_485_Driver_;
    
    int i;
    int nRet = 0;
    // uint8_t sendmsg[14] = {0};
    // sendmsg[0] = 0x3E;
    // sendmsg[1] = 0xA3;
    // sendmsg[2] = 0x01;
    // sendmsg[3] = 0x08;
    // sendmsg[4] = Checksumcrc(sendmsg,0,4);
    // sendmsg[5] = 0x05;
    // sendmsg[6] = 0x06;
    // sendmsg[7] = 0x07;
    // sendmsg[8] = 0x08;
    // sendmsg[9] = 0x09;
    // sendmsg[10] = 0x0A;
    // sendmsg[11] = 0x0B;
    // sendmsg[12] = 0x0C;
    // sendmsg[13] = Checksumcrc(sendmsg,5,8);


    // std::cout<<"0000000="<<std::endl; 
    // for(int i=0; i<14; i++)
    // {
    //     printf("%x\n",sendmsg[i]);
    // }

    // uint8_t sendmsg[5] = {0};

    // sendmsg[0] = 0x3E;
    // sendmsg[1] = 0x80;    //81
    // sendmsg[2] = 0x01;
    // sendmsg[3] = 0x00;   //4
    // sendmsg[4] = Checksumcrc(sendmsg,0,4);


    uint8_t Motor_ID = {0x01};
    int64_t angleControl_1 = 15;
    uint8_t spinDirection = {0x01};

    std::vector<uint8_t> p;

    // RMD_485_Driver_.RS_angleControl_1(Motor_ID, angleControl_1, p);
    // RMD_485_Driver_.RS_angleControl_3(Motor_ID, spinDirection, angleControl_1, p);
    RMD_485_Driver_.RS_Motor_off(Motor_ID, p);
    // RMD_485_Driver_.RS_Motor_stop(Motor_ID, p);

    int size = p.size();
    std::cout<<"size="<<size<<std::endl;
    uint8_t sendmsg[size]={0};
    for(i =0; i<size; i++)
    {
        sendmsg[i] = p[i];
        printf("%x\n",sendmsg[i]);

    }
    p.clear();
    std::cout<<"0000000="<<std::endl; 

    int SIZE = sizeof(sendmsg);
    std::cout<<"SIZE="<<SIZE<<std::endl;

    uint8_t buf[SIZE];
    if( SerialInit() == -1 )
    {
        perror("SerialInit Error!\n");
        return -1;
    }
    bzero(buf, SIZE);

    // // while(1)
    {   sleep(1);
        write(nFd,sendmsg,sizeof(sendmsg));//向串口发送数据
        printf("%s\n",sendmsg);

        /////////////串口接收部分////////////////
        nRet = read(nFd, buf, SIZE);
        if(-1 == nRet)
        {
            perror("Read Data Error!\n");
            // break;
        }
        if(0 < nRet)
        {
            buf[nRet] = 0;
            printf("Recv Data: %s\n", buf);
        }
        ////////////////////////////////////////
    }
    close(nFd);
    return 0;
}


// #include <stdio.h>
// #include <fcntl.h>       /* File Control Definitions           */
// #include <termios.h>     /* POSIX Terminal Control Definitions */
// #include <unistd.h>      /* UNIX Standard Definitions 	       */ 
// #include <errno.h>       /* ERROR Number Definitions           */
// #include <sys/ioctl.h>   /* ioctl()                            */

// int main( )
// {
//     int fd;/*File Descriptor*/

// printf("\n +----------------------------------+");
// printf("\n |        USB To RS485 Write        |");
// printf("\n +----------------------------------+");

// /*------------------------------- Opening the Serial Port -------------------------------*/

// /* Change /dev/ttyUSB0 to the one corresponding to your system */

// fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY);	/* ttyUSB0 is the FT232 based USB2SERIAL Converter   */
//                         /* O_RDWR Read/Write access to serial port           */
//                         /* O_NOCTTY - No terminal will control the process   */
                                                        
                        
// if(fd == -1)						/* Error Checking */
//         printf("\n  Error! in Opening ttyUSB0  ");
// else
//         printf("\n  ttyUSB0 Opened Successfully ");


// /*---------- Setting the Attributes of the serial port using termios structure --------- */

// struct termios SerialPortSettings;	/* Create the structure                          */

// tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

// cfsetispeed(&SerialPortSettings,B115200); /* Set Read  Speed as 9600                       */
// cfsetospeed(&SerialPortSettings,B115200); /* Set Write Speed as 9600                       */

// SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
// SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
// SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
// SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */

// SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
// SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */ 


// SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
// SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

// SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

// if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
//     printf("\n  ERROR ! in Setting attributes");
// else
//     printf("\n  BaudRate = 115200 \n  StopBits = 1 \n  Parity   = none");

// /*---------------------------------------- Controlling RTS and DTR Pins --------------------------------------*/
// /* ~RTS(USB2SERIAL) ---> ~RE(MAX485) */
// /* ~DTR(USB2SERIAL) --->  DE(MAX485) */

// /*------------------------- Putting MAX485 chip in USB2SERIAL in Transmit Mode ---------------------------*/
// //                                                                                                        //
// //	----+			+-----------+              H  +-----------+                                           //
// //		|			| 	    ~RTS| --------------> |~RE        |                                           //
// //	 PC |==========>| FT232     |                 |   MAX485  +(A,B)~~~~~~~~~~~~~~~>Data out(RS485)       //
// //	    |    USB    |       ~DTR| --------------> | DE        |        Twisted Pair                       //
// //  ----+			+-----------+              H  +-----------+                                           //
// //                                                                                                        //
// //--------------------------------------------------------------------------------------------------------//
// //TxMode - DE->High,~RE -> High

// int RTS_flag,DTR_flag;

// RTS_flag = TIOCM_RTS;	/* Modem Constant for RTS pin */
// DTR_flag = TIOCM_DTR;	/* Modem Constant for DTR pin */

// ioctl(fd,TIOCMBIC,&RTS_flag);/* ~RTS = 1,So ~RE pin of MAX485 is HIGH                       */
// ioctl(fd,TIOCMBIC,&DTR_flag);/* ~DTR = 1,So  DE pin of MAX485 is HIGH,Transmit Mode enabled */ 

//     /*------------------------------- Write data to serial port -----------------------------*/

// char write_buffer[] = "3E A3 01 08 EA 05 06 07 08 09 0A 0B 0C 44";	/* Buffer containing characters to write into port	     */	
// int  bytes_written  = 0;  	/* Value for storing the number of bytes written to the port */ 

// bytes_written = write(fd,write_buffer,sizeof(write_buffer));/* use write() to send data to port                                            */
//                                     /* "fd"                   - file descriptor pointing to the opened serial port */
//                                     /*	"write_buffer"         - address of the buffer containing data	            */
//                                     /* "sizeof(write_buffer)" - No of bytes to write                               */	
// printf("\n  %s written to ttyUSB0",write_buffer);
// printf("\n  %d Bytes written to ttyUSB0", bytes_written);
// printf("\n +----------------------------------+\n\n");

// close(fd);/* Close the Serial port */
// return 0;
// }

