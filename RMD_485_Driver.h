#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <vector>

class RMD_485_Driver
{
    public:
    uint8_t Checksumcrc(uint8_t *aData, uint8_t StartIndex, uint8_t DataLength);
    void RS_powerControl(uint8_t Motor_ID, int16_t powerControl);
    void RS_iqControl(uint8_t Motor_ID, int32_t iqControl);
    void RS_speedControl(uint8_t Motor_ID, int32_t speedControl);
    // void RS_angleControl_1(uint8_t Motor_ID, int64_t angleControl_1);
    void RS_angleControl_1(uint8_t Motor_ID, int64_t angleControl_1,  std::vector<uint8_t> &Data);
    void RS_angleControl_3(uint8_t Motor_ID, uint8_t spinDirection, int16_t angleControl_1, std::vector<uint8_t> &Data);
    void RS_Motor_off(uint8_t Motor_ID, std::vector<uint8_t> &Data);
    void RS_Motor_stop(uint8_t Motor_ID, std::vector<uint8_t> &Data);
    void RS_Motor_start(uint8_t Motor_ID);

};
