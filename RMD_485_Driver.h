#include <iostream>
#include <stdint.h>
#include <vector>
#include <string.h>
// 串口通讯头文件
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define DEVICE "/dev/ttyUSB1"

class RMD_485_Driver {
  public:
  uint8_t Checksumcrc(uint8_t *aData, uint8_t StartIndex, uint8_t DataLength);
  void RS_powerControl(uint8_t Motor_ID, int16_t powerControl);
  void RS_iqControl(uint8_t Motor_ID, int32_t iqControl);
  void RS_speedControl(uint8_t Motor_ID, int32_t speedControl);
  // void RS_angleControl_1(uint8_t Motor_ID, int64_t angleControl_1);
  void RS_angleControl_1(uint8_t Motor_ID, int64_t angleControl_1);
  void RS_angleControl_3(uint8_t Motor_ID, uint8_t spinDirection, int16_t angleControl_1, std::vector<uint8_t> &Data);
  void RS_Motor_off(uint8_t Motor_ID, std::vector<uint8_t> &Data);
  void RS_Motor_stop(uint8_t Motor_ID, std::vector<uint8_t> &Data);
  void RS_Motor_start(uint8_t Motor_ID);
  int SerialInit();
  void SerialPush(std::vector<uint8_t> &p);
};


