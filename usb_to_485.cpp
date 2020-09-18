#include "RMD_485_Driver.h"
#include <climits>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#define BAUDRATE B115200
#define DEVICE "/dev/ttyUSB0"


int nFd = 0;
struct termios SerialSettings;
struct termios SerialOld;

// Initialize serial port
int SerialInit() {
  /* *** 打开串口 *** */
  // O_RDWR: 读写方式打开; O_NOCTTY: 不允许进程管理串口; O_NDELAY: 非阻塞
  nFd = open(DEVICE, O_RDWR|O_NOCTTY|O_NDELAY);
  if(-1 == nFd) {
    perror("Open Serial Port Error!\n");
    return -1;
  }
  // 回复串口为阻塞状态
  if ((fcntl(nFd, F_SETFL, 0)) < 0) {
    perror("Fcntl F_SETFL Error!\n");
    return -1;
  }
  // 测试是否为终端设备
  if(isatty(STDIN_FILENO)==0) {
    printf("standard input is not a terminal device\n");
    return -1;
  }
  // 保存原先串口的配置
  if (tcgetattr(nFd, &SerialOld) != 0) {
    perror("tcgetattr error!\n");
    return -1;
  }

  /* *** 串口参数设置 *** */
  // 获取串口原来的参数设置
  tcgetattr(nFd, &SerialSettings);
  // 设置终端为原始模式，该模式下全部的输入数据以字节为单位被处理
  cfmakeraw(&SerialSettings);
  // 设置波特率，用户不能直接通过位掩码来操作
  cfsetispeed(&SerialSettings, B115200);
  cfsetospeed(&SerialSettings, B115200);

  // 本地连接 | 接收使能
  SerialSettings.c_cflag |= (CLOCAL|CREAD);
  // 用数据位掩码清空数据位设置
  SerialSettings.c_cflag &= ~CSIZE;
  // 数据位为8位
  SerialSettings.c_cflag |= CS8;
  // 无校验位
  SerialSettings.c_cflag &= ~PARENB;
  // 无奇偶校验位
  SerialSettings.c_iflag &= ~INPCK; 
  // 清除CSTOPB，设置停止位为1 bits
  SerialSettings.c_cflag &= ~CSTOPB;

  // 设置read()函数的调用方式
  // 指定读取每个字符之间的超时时间
  SerialSettings.c_cc[VTIME]=0;
  // 指定所要读取字符的最小数量
  SerialSettings.c_cc[VMIN]=1;

  // 清空终端未完毕的输入/输出请求及数据
  tcflush(nFd,TCIFLUSH);
  // 立即激活新配置
  if (tcsetattr(nFd, TCSANOW, &SerialSettings) != 0) {
    perror("tcsetattr Error!\n");
    return -1;
  }
  return nFd;
}

uint8_t Checksumcrc(uint8_t *aData, uint8_t StartIndex, uint8_t DataLength) {
  uint8_t crc = 0;
  uint8_t i = 0;
  for(i=StartIndex; i<(StartIndex + DataLength); i++)
  {
   crc +=  aData[i] ;	  
  }	
  return crc;
}

int main(int argc, char **argv) {
  RMD_485_Driver RMD_485_Driver_;

  int i;
  int nRet = 0;
  uint8_t Motor_ID = {0x01};
  int64_t angleControl_1 = 15;
  uint8_t spinDirection = {0x01};

  std::vector<uint8_t> p;

  // RMD_485_Driver_.RS_angleControl_1(Motor_ID, angleControl_1, p);
  // RMD_485_Driver_.RS_angleControl_3(Motor_ID, spinDirection, angleControl_1,
  // p);
  RMD_485_Driver_.RS_Motor_off(Motor_ID, p);
  // RMD_485_Driver_.RS_Motor_stop(Motor_ID, p);

  int size = p.size();
  std::cout << "size=" << size << std::endl;
  uint8_t sendmsg[size];
  for (i = 0; i < size; i++) {
    sendmsg[i] = p[i];
    printf("%x\n", sendmsg[i]);
  }
  p.clear();
  std::cout << "0000000=" << std::endl;

  int SIZE = sizeof(sendmsg);
  std::cout << "SIZE=" << SIZE << std::endl;

  uint8_t buf[SIZE];
  if (SerialInit() == -1) {
    perror("SerialInit Error!\n");
    return -1;
  }
  bzero(buf, SIZE);

  // // while(1)
  {
    sleep(1);
    write(nFd, sendmsg, sizeof(sendmsg)); //向串口发送数据
    printf("%s\n", sendmsg);

    /////////////串口接收部分////////////////
    nRet = read(nFd, buf, SIZE);
    if (-1 == nRet) {
      perror("Read Data Error!\n");
      // break;
    }
    if (0 < nRet) {
      buf[nRet] = 0;
      printf("Recv Data: %s\n", buf);
    }
    ////////////////////////////////////////
  }
  close(nFd);
  return 0;
}

