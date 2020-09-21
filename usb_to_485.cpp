/*******************************************
 * TODO: 
 * 1. 指针/数组: 指令简化
 *******************************************/

#include "RMD_485_Driver.h"
#include <climits>
#include <errno.h>
#include <stdint.h>
#include <string.h>

extern int nFd;

int main(int argc, char **argv) {
  RMD_485_Driver RMD_485_Driver_;

  int i;
  std::vector<uint8_t> p;
  uint8_t Motor_ID = {0x02};
  int64_t angleControl_1 = 15;
  uint8_t spinDirection = {0x01};

  // 初始化串口
  if (RMD_485_Driver_.SerialInit() == -1) {
    perror("SerialInit Error!\n");
    return -1;
  }

  std::cout<<"angleControl_1="<<angleControl_1<<std::endl; 
  RMD_485_Driver_.RS_angleControl_1(Motor_ID, angleControl_1);

  sleep(1);
  RMD_485_Driver_.RS_Motor_off(Motor_ID);

  close(nFd);
  return 0;
}

