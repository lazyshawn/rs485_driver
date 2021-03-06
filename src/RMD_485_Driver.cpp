/**
  * @brief  send a angle control frame via RS485 bus
  * @param  motor ID ,1-32
  * @param  power value ,power range -1000~ 1000
  * @retval null
  */
//
#include "RMD_485_Driver.h"

int nFd = 0;
struct termios SerialSettings;
struct termios SerialOld;

// Initialize serial port
int RMD_485_Driver::SerialInit() {
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


/* *** 发送电机指令 *** */
void RMD_485_Driver::SerialPush(uint8_t *Tx, uint8_t *Rx, int size[]) {

  int i;
  int nRet = 0;
  int Tx_len = size[0], Rx_len = size[1];
  uint8_t sendmsg[Tx_len], rcvmsg[Rx_len];

  /* *** 串口发送部分 *** */
  for (i = 0; i < Tx_len; i++) {
    sendmsg[i] = *Tx;
    printf("sendmsg[%d] = %x\n",i,sendmsg[i]);
    Tx++;
  }
  write(nFd, sendmsg, Tx_len);
  /* === 串口发送部分 === */

  /* *** 串口接收部分 *** */
  nRet = read(nFd, rcvmsg, Rx_len);
  if (-1 == nRet) {
    perror("Read Data Error!\n");
  }
  else {
    rcvmsg[nRet] = 0;
    for (i=0; i<Rx_len; i++){
      *Rx = rcvmsg[i];
      Rx++;
    }
  }
  /* === 串口接收部分 === */
}


/* *** 计算校验和 *** */
uint8_t RMD_485_Driver::Checksumcrc(uint8_t *aData, 
    uint8_t StartIndex, uint8_t DataLength) {

  uint8_t crc = 0;
  uint8_t i = 0;
  for(i=StartIndex; i<(StartIndex + DataLength); i++) {
   crc += aData[i];
  }
  return crc;
}


/** 开环控制命令 **/
void RMD_485_Driver::RS_powerControl(
    uint8_t Motor_ID, int16_t powerControl) {

  uint8_t TxData[8] = {0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0xA0;
  TxData[2] = Motor_ID;
  TxData[3] = 2;
  TxData[4] = RMD_485_Driver::Checksumcrc(TxData,0,4);
  TxData[5] = *((uint8_t *)(&powerControl));
  TxData[6] = *((uint8_t *)(&powerControl)+1);
  TxData[7] = RMD_485_Driver::Checksumcrc(TxData,5,2);
        
}


/**
  * @brief  send a iqControl control frame via RS485 bus
  * @param  motor ID ,1-32
  * @param  power value ,power range -1000~ 1000,iqControl range -2000~ 2000 ,-32A~32A
  * @retval null
  */
/** 转矩闭环控制命令 **/
void RMD_485_Driver::RS_iqControl(
    uint8_t Motor_ID, int32_t iqControl) {

  uint8_t TxData[8] = {0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0xA1;
  TxData[2] = Motor_ID;
  TxData[3] = 2;
  TxData[4] = RMD_485_Driver::Checksumcrc(TxData,0,4);
  TxData[5] = *((uint8_t *)(&iqControl));
  TxData[6] = *((uint8_t *)(&iqControl)+1);
  TxData[7] = RMD_485_Driver::Checksumcrc(TxData,5,2);
        
}


/**
  * @brief  send a speed control frame via RS485 bus
  * @param  motor ID ,1-32
  * @param  speed,  0.01dps
  * @retval null
  */
/** 速度闭环控制命令 **/
void RMD_485_Driver::RS_speedControl(
    uint8_t Motor_ID, int32_t speedControl) {

  uint8_t TxData[10] = {0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0xA2;
  TxData[2] = Motor_ID;
  TxData[3] = 4;
  TxData[4] = RMD_485_Driver::Checksumcrc(TxData,0,4);
  TxData[5] = *((uint8_t *)(&speedControl));
  TxData[6] = *((uint8_t *)(&speedControl)+1);
  TxData[7] = *((uint8_t *)(&speedControl)+2);
  TxData[8] = *((uint8_t *)(&speedControl)+3);
  TxData[9] = RMD_485_Driver::Checksumcrc(TxData,5,4);
        
}


/** 多圈位置闭环控制命令 1 **/
void RMD_485_Driver::RS_angleControl_1(
    uint8_t Motor_ID, int64_t angleControl_1) {

  int i;
  int size[2] = {14, 13};
  uint8_t TxData[14] = {0};
  uint8_t RxData[13] = {0};

  TxData[0] = 0x3E;
  TxData[1] = 0xA3;
  TxData[2] = Motor_ID;
  TxData[3] = 8;
  TxData[4] = RMD_485_Driver::Checksumcrc(TxData,0,4);
  TxData[5] = *(uint8_t *)(&angleControl_1); 
  TxData[6] = *(uint8_t *)(&angleControl_1)+1;
  TxData[7] = *(uint8_t *)(&angleControl_1)+2;
  TxData[8] = *(uint8_t *)(&angleControl_1)+3;
  TxData[9] = *(uint8_t *)(&angleControl_1)+4;
  TxData[10] = *(uint8_t *)(&angleControl_1)+5;
  TxData[11] = *(uint8_t *)(&angleControl_1)+6;
  TxData[12] = *(uint8_t *)(&angleControl_1)+7;
  TxData[13] = RMD_485_Driver::Checksumcrc(TxData,5,8);
        
  RMD_485_Driver::SerialPush(TxData, RxData, size);

  for (i = 0; i < size[1]; i++) {
    printf("rcvmsg[%d] = %x\n",i,RxData[i]);
  }
}


//single loop1
//spinDirection 0x00 clockwise, 0x01 counter clockwise;
/** 单圈位置闭环控制命令 1 **/
void RMD_485_Driver::RS_angleControl_3(
    uint8_t Motor_ID, uint8_t spinDirection, int16_t angleControl_1, std::vector<uint8_t> &Data) {

  uint8_t TxData[10] = {0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0xA5;
  TxData[2] = Motor_ID;
  TxData[3] = 4;
  TxData[4] = RMD_485_Driver::Checksumcrc(TxData,0,4);
  TxData[5] = spinDirection;
  TxData[6] = *(uint8_t *)(&angleControl_1);
  TxData[7] = *(uint8_t *)(&angleControl_1)+1;
  TxData[8] = 0;
  TxData[9] = RMD_485_Driver::Checksumcrc(TxData,5,4);
        
}


/** 电机停止命令 **/
void RMD_485_Driver::RS_Motor_stop(
    uint8_t Motor_ID, std::vector<uint8_t> &Data) {

  uint8_t TxData[5] = {0};

  TxData[0] = 0x3E;
  TxData[1] = 0x81;
  TxData[2] = Motor_ID;
  TxData[3] = 4;
  TxData[4] = RMD_485_Driver::Checksumcrc(TxData,0,4);   

}


/** 电机关闭命令 **/
void RMD_485_Driver::RS_Motor_off(uint8_t Motor_ID){

  int i;
  int size[2] = {5,5};
  uint8_t TxData[5] = {0};
  uint8_t RxData[5] = {0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0x80;
  TxData[2] = Motor_ID;
  TxData[3] = 0x00;
  TxData[4] = RMD_485_Driver::Checksumcrc(TxData,0,4);
        
  RMD_485_Driver::SerialPush(TxData, RxData, size);

  for (i = 0; i < size[1]; i++) {
    printf("rcvmsg[%d] = %x\n",i,RxData[i]);
  }
}


/** 电机开始命令 **/
void RMD_485_Driver::RS_Motor_start(uint8_t Motor_ID) {

  uint8_t TxData[5] = {0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0x88;
  TxData[2] = Motor_ID;
  TxData[3] = 4;
  TxData[4] = RMD_485_Driver::Checksumcrc(TxData,0,4);
        
}

