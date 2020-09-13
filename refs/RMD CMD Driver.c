

/**
  * @brief  send a Torque control frame via CAN bus
  * @param  motor ID ,1-32
  * @param  iq with the value range of -2000~ 2000, corresponding to the actual torque current range of -32A ~32A
  * @retval null
  */
void iqControl(uint8_t Motor_ID, int32_t iqControl)
{
  uint8_t TxData[8] = {0};
  
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;            
  TxHeader.StdId = std_id + Motor_ID;
  TxHeader.TransmitGlobalTime = DISABLE;
  TxHeader.DLC = 8;
    
  TxData[0] = 0xA1;
  TxData[1] = 0;
  TxData[2] = 0;
  TxData[3] = 0;
  TxData[4] = *(uint8_t *)(&iqControl); 
  TxData[5] = *((uint8_t *)(&iqControl)+1);
  TxData[6] = 0;
  TxData[7] = 0;
        
 if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
 {
   /* Transmission request Error */
   Error_Handler();
 }
}

/**
  * @brief  send a speed control frame via CAN bus
  * @param  motor ID ,1-32
  * @param  int32_t corresponding to the actual speed of 0.01 DPS /LSB.
  * @retval null
  */
void speedControl(uint8_t Motor_ID, int32_t speedControl)
{
  uint8_t TxData[8] = {0};
  
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;            
  TxHeader.StdId = std_id + Motor_ID;
  TxHeader.TransmitGlobalTime = DISABLE;
  TxHeader.DLC = 8;
    
  TxData[0] = 0xA2;
  TxData[1] = 0;
  TxData[2] = 0;
  TxData[3] = 0;
  TxData[4] = *(uint8_t *)(&speedControl); //位置控制低字节
  TxData[5] = *((uint8_t *)(&speedControl)+1);
  TxData[6] = *((uint8_t *)(&speedControl)+2);
  TxData[7] = *((uint8_t *)(&speedControl)+3);//位置控制高字节
        
 if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
 {
   /* Transmission request Error */
   Error_Handler();
 }

}

/**
  * @brief  send a angle control frame via CAN bus
  * @param  motor ID ,1-32
  * @param  the actual position is 0.01degree/LSB, 36000 represents 360°
  * @retval null
  */
void Multi_angleControl_1(uint8_t Motor_ID, int32_t angleControl)
{
  uint8_t TxData[8] = {0};
  
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;            
  TxHeader.StdId = std_id + Motor_ID;
  TxHeader.TransmitGlobalTime = DISABLE;
  TxHeader.DLC = 8;
    
  TxData[0] = 0xA3;
  TxData[1] = 0;
  TxData[2] = 0;
  TxData[3] = 0;
  TxData[4] = *(uint8_t *)(&angleControl); //位置控制低字节
  TxData[5] = *((uint8_t *)(&angleControl)+1);
  TxData[6] = *((uint8_t *)(&angleControl)+2);
  TxData[7] = *((uint8_t *)(&angleControl)+3);//位置控制高字节
        
 if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
 {
   /* Transmission request Error */
   Error_Handler();
 }

}


void Multi_angleControl_2(uint8_t Motor_ID, uint16_t maxSpeed, int32_t angleControl)
{

  uint8_t TxData[8] = {0};
  
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;            
  TxHeader.StdId = std_id + Motor_ID;
  TxHeader.TransmitGlobalTime = DISABLE;
  TxHeader.DLC = 8;
    
  TxData[0] = 0xA4;
  TxData[1] = 0;
  TxData[2] = *(uint8_t *)(&maxSpeed);//速度限制低字节
  TxData[3] = *(uint8_t *)((&maxSpeed)+1);//速度限制高字节
  TxData[4] = *(uint8_t *)(&angleControl); //位置控制低字节
  TxData[5] = *((uint8_t *)(&angleControl)+1);
  TxData[6] = *((uint8_t *)(&angleControl)+2);
  TxData[7] = *((uint8_t *)(&angleControl)+3);//位置控制高字节
  
  //HAL_UART_Transmit(&huart1, TxData, 8, 10);
  
  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
    /* Transmission request Error */
    Error_Handler();
  }

}

void Single_loop_angleControl_1(uint8_t Motor_ID, uint8_t spinDirection, uint16_t angleControl)
{

  uint8_t TxData[8] = {0};
  
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;            
  TxHeader.StdId = std_id + Motor_ID;
  TxHeader.TransmitGlobalTime = DISABLE;
  TxHeader.DLC = 8;
    
  TxData[0] = 0xA5;
  TxData[1] = spinDirection;
  TxData[2] = 0;
  TxData[3] = 0;
  TxData[4] = *(uint8_t *)(&angleControl); 
  TxData[5] = *((uint8_t *)(&angleControl)+1);
  TxData[6] = 0;
  TxData[7] = 0;
        
 if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
 {
   /* Transmission request Error */
   Error_Handler();
 }

}

void Single_loop_angleControl_2(uint8_t Motor_ID, uint8_t spinDirection, uint16_t maxSpeed, uint16_t angleControl)
{

  uint8_t TxData[8] = {0};
  
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;            
  TxHeader.StdId = std_id + Motor_ID;
  TxHeader.TransmitGlobalTime = DISABLE;
  TxHeader.DLC = 8;
    
  TxData[0] = 0xA6;
  TxData[1] = spinDirection;
  TxData[2] = *(uint8_t *)(&maxSpeed);
  TxData[3] = *((uint8_t *)(&maxSpeed)+1);
  TxData[4] = *(uint8_t *)(&angleControl); 
  TxData[5] = *((uint8_t *)(&angleControl)+1);
  TxData[6] = 0;
  TxData[7] = 0;
        
 if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
 {
   /* Transmission request Error */
   Error_Handler();
 }

}


/**
  * @brief  send a angle control frame via CAN bus
  * @param  motor ID ,1-32
  * @param  ACC
  * @retval null
  */
void Multi_Wr_Accel_Ram(uint8_t Motor_ID, int32_t Accel)
{
  uint8_t TxData[8] = {0};
  
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;            
  TxHeader.StdId = std_id + Motor_ID;
  TxHeader.TransmitGlobalTime = DISABLE;
  TxHeader.DLC = 8;
    
  TxData[0] = 0xA3;
  TxData[1] = 0;
  TxData[2] = 0;
  TxData[3] = 0;
  TxData[4] = *(uint8_t *)(&Accel); //位置控制低字节
  TxData[5] = *((uint8_t *)(&Accel)+1);
  TxData[6] = *((uint8_t *)(&Accel)+2);
  TxData[7] = *((uint8_t *)(&Accel)+3);//位置控制高字节
        
 if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
 {
   /* Transmission request Error */
   Error_Handler();
 }

}

/**
  * @brief  motor off frame via CAN bus
  * @param  motor ID ,1-32
  * @retval null
  */
void Motor_Off(uint8_t Motor_ID)
{

  uint8_t TxData[8] = {0};
  
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;            
  TxHeader.StdId= std_id + Motor_ID;
  TxHeader.TransmitGlobalTime = DISABLE;
  TxHeader.DLC = 8;
    
  TxData[0] = 0x81;
  TxData[1] = 0;
  TxData[2] = 0;
  TxData[3] = 0;
  TxData[4] = 0;
  TxData[4] = 0;
  TxData[5] = 0;
  TxData[6] = 0;
  TxData[7] = 0;
        
 if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
 {
   /* Transmission request Error */
   Error_Handler();
 }

}

/**
  * @brief  send a angle control frame via RS485 bus
  * @param  motor ID ,1-32
  * @param  power value ,power range -1000~ 1000
  * @retval null
  */
//
void RS_powerControl(uint8_t Motor_ID, int16_t powerControl)
{
  uint8_t TxData[8] = {0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0xA0;
  TxData[2] = Motor_ID;
  TxData[3] = 2;
  TxData[4] = Checksumcrc(TxData,0,4);
  TxData[5] = *((uint8_t *)(&powerControl));
  TxData[6] = *((uint8_t *)(&powerControl)+1);
  TxData[7] = Checksumcrc(TxData,5,2);
        
  HAL_UART_Transmit(&huart5, TxData, 8, 10);

}

/**
  * @brief  send a iqControl control frame via RS485 bus
  * @param  motor ID ,1-32
  * @param  power value ,power range -1000~ 1000,iqControl range -2000~ 2000 ,-32A~32A
  * @retval null
  */
void RS_iqControl(uint8_t Motor_ID, int32_t iqControl)
{
  uint8_t TxData[8] = {0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0xA1;
  TxData[2] = Motor_ID;
  TxData[3] = 2;
  TxData[4] = Checksumcrc(TxData,0,4);
  TxData[5] = *((uint8_t *)(&iqControl));
  TxData[6] = *((uint8_t *)(&iqControl)+1);
  TxData[7] = Checksumcrc(TxData,5,2);
        
  HAL_UART_Transmit(&huart5, TxData, 8, 10);

}

/**
  * @brief  send a speed control frame via RS485 bus
  * @param  motor ID ,1-32
  * @param  speed,  0.01dps
  * @retval null
  */
void RS_speedControl(uint8_t Motor_ID, int32_t speedControl)
{
  uint8_t TxData[10] = {0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0xA2;
  TxData[2] = Motor_ID;
  TxData[3] = 4;
  TxData[4] = Checksumcrc(TxData,0,4);
  TxData[5] = *((uint8_t *)(&speedControl));
  TxData[6] = *((uint8_t *)(&speedControl)+1);
  TxData[7] = *((uint8_t *)(&speedControl)+2);
  TxData[8] = *((uint8_t *)(&speedControl)+3);
  TxData[9] = Checksumcrc(TxData,5,4);
        
  HAL_UART_Transmit(&huart5, TxData, 10, 10);

}

/**
  * @brief  send a multi loop angle control frame via RS485 bus
  * @param  motor ID ,1-32
  * @param  multi loop angle
  * @retval null
  */
void RS_angleControl_1(uint8_t Motor_ID, int64_t angleControl_1)
{
  uint8_t TxData[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0xA3;
  TxData[2] = Motor_ID;
  TxData[3] = 8;
  TxData[4] = Checksumcrc(TxData,0,4);
  TxData[5] = *((uint8_t *)(&angleControl_1)); //*(uint8_t *)(&angleControl);
  TxData[6] = *((uint8_t *)(&angleControl_1)+1);
  TxData[7] = *((uint8_t *)(&angleControl_1)+2);
  TxData[8] = *((uint8_t *)(&angleControl_1)+3);
  TxData[9] = *((uint8_t *)(&angleControl_1)+4);
  TxData[10] = *((uint8_t *)(&angleControl_1)+5);
  TxData[11] = *((uint8_t *)(&angleControl_1)+6);
  TxData[12] = *((uint8_t *)(&angleControl_1)+7);
  TxData[13] = Checksumcrc(TxData,5,8);
        
  HAL_UART_Transmit(&huart5, TxData, 14, 10);
  
  uint8_t i=0;
  for (i=0;i<14;i++)
    printf("buf %d = %d\r\n",i,TxData[i]);

}

/**
  * @brief  send a loop angle control with speed limit frame via RS485 bus
  * @param  motor ID ,1-32
  * @param  multi loop angle
  * @param  maxSpeed
  * @retval null
  */
void RS_angleControl_2(uint8_t Motor_ID, int64_t angleControl_1, uint32_t maxSpeed)
{
  uint8_t TxData[18] = {0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0xA4;
  TxData[2] = Motor_ID;
  TxData[3] = 12;
  TxData[4] = Checksumcrc(TxData,0,4);
  TxData[5] = *((uint8_t *)(&angleControl_1));
  TxData[6] = *((uint8_t *)(&angleControl_1)+1);
  TxData[7] = *((uint8_t *)(&angleControl_1)+2);
  TxData[8] = *((uint8_t *)(&angleControl_1)+3);
  TxData[9] = *((uint8_t *)(&angleControl_1)+4);
  TxData[10] = *((uint8_t *)(&angleControl_1)+5);
  TxData[11] = *((uint8_t *)(&angleControl_1)+6);
  TxData[12] = *((uint8_t *)(&angleControl_1)+7);
  TxData[13] = *((uint8_t *)(&maxSpeed));
  TxData[14] = *((uint8_t *)(&maxSpeed)+1);
  TxData[15] = *((uint8_t *)(&maxSpeed)+2);
  TxData[16] = *((uint8_t *)(&maxSpeed)+3);
  TxData[17] = Checksumcrc(TxData,5,12);
        
  HAL_UART_Transmit(&huart5, TxData, 18, 10);

}

//single loop1
void RS_angleControl_3(uint8_t Motor_ID, uint8_t spinDirection, int16_t angleControl_1)
{
  uint8_t TxData[10] = {0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0xA5;
  TxData[2] = Motor_ID;
  TxData[3] = 4;
  TxData[4] = Checksumcrc(TxData,0,4);
  TxData[5] = spinDirection;
  TxData[6] = *((uint8_t *)(&angleControl_1));
  TxData[7] = *((uint8_t *)(&angleControl_1)+1);
  TxData[8] = 0;
  TxData[9] = Checksumcrc(TxData,5,4);
        
  HAL_UART_Transmit(&huart5, TxData, 10, 10);

}

//single loop2
void RS_angleControl_4(uint8_t Motor_ID, uint8_t spinDirection, uint16_t angleControl_1, uint32_t maxSpeed)
{
  uint8_t TxData[14] = {0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0xA6;
  TxData[2] = Motor_ID;
  TxData[3] = 8;
  TxData[4] = Checksumcrc(TxData,0,4);
  TxData[5] = spinDirection;
  TxData[6] = *((uint8_t *)(&angleControl_1));
  TxData[7] = *((uint8_t *)(&angleControl_1)+1);
  TxData[8] = 0;
  TxData[9] = *((uint8_t *)(&maxSpeed));
  TxData[10] = *((uint8_t *)(&maxSpeed)+1);
  TxData[11] = *((uint8_t *)(&maxSpeed)+2);
  TxData[12] = *((uint8_t *)(&maxSpeed)+3);
  TxData[13] = Checksumcrc(TxData,5,8);
        
  HAL_UART_Transmit(&huart5, TxData, 14, 10);
  
  /*
  uint8_t i=0;
  for (i=0;i<14;i++)
  printf("buf%d=%#X\r\n",i,TxData[i]);
  */
}

//angleIncrement1 
void RS_angleControl_5(uint8_t Motor_ID,  int32_t angleIncrement)
{
  uint8_t TxData[10] = {0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0xA7;
  TxData[2] = Motor_ID;
  TxData[3] = 4;
  TxData[4] = Checksumcrc(TxData,0,4);
  TxData[5] = *((uint8_t *)(&angleIncrement));
  TxData[6] = *((uint8_t *)(&angleIncrement)+1);
  TxData[7] = *((uint8_t *)(&angleIncrement)+2);
  TxData[8] = *((uint8_t *)(&angleIncrement)+3);
  TxData[9] = Checksumcrc(TxData,5,4);
        
  HAL_UART_Transmit(&huart5, TxData, 10, 10);

}

//angleIncrement2
void RS_angleControl_6(uint8_t Motor_ID,  int32_t angleIncrement, uint32_t maxSpeed)
{
  uint8_t TxData[14] = {0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0xA8;
  TxData[2] = Motor_ID;
  TxData[3] = 8;
  TxData[4] = Checksumcrc(TxData,0,4);
  TxData[5] = *((uint8_t *)(&angleIncrement));
  TxData[6] = *((uint8_t *)(&angleIncrement)+1);
  TxData[7] = *((uint8_t *)(&angleIncrement)+2);
  TxData[8] = *((uint8_t *)(&angleIncrement)+3);
  TxData[9] = *((uint8_t *)(&maxSpeed));
  TxData[10] = *((uint8_t *)(&maxSpeed)+1);
  TxData[11] = *((uint8_t *)(&maxSpeed)+2);
  TxData[12] = *((uint8_t *)(&maxSpeed)+3);
  TxData[13] = Checksumcrc(TxData,5,8);
        
  HAL_UART_Transmit(&huart5, TxData, 14, 10);

}

/**
  * @brief  send a Motor_Off frame via RS485 bus
  * @param  motor ID ,1-32
  * @retval null
  */
void RS_Motor_Off(uint8_t Motor_ID)
{
  uint8_t TxData[5] = {0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0x80;
  TxData[2] = Motor_ID;
  TxData[3] = 4;
  TxData[4] = Checksumcrc(TxData,0,4);
        
  HAL_UART_Transmit(&huart5, TxData, 5, 10);

}

/**
  * @brief  Checksumcrc
  * @param  buf start byte
  * @param  crc length.
  * @retval Checksumcrc.
  */
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