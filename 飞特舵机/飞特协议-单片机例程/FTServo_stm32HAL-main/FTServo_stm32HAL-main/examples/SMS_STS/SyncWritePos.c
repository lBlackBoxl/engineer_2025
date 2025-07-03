#include "main.h"
#include <../../SCSLib/SCServo.h>

uint8_t ID[2];
int16_t Position[2];
uint16_t Speed[2];
uint8_t ACC[2];

void setup(void)
{
	setEnd(0);//SMS_STS舵机为大端存储结构
  ID[0] = 1;//舵机ID1
  ID[1] = 2;//舵机ID2
  Speed[0] = 60;//最高速度V=60*0.732=43.92rpm
  Speed[1] = 60;//最高速度V=60*0.732=43.92rpm
  ACC[0] = 50;//加速度A=50*8.7deg/s^2
  ACC[1] = 50;//加速度A=50*8.7deg/s^2
}

void examples(void)
{
  //舵机(ID1/ID2)以最高速度V=60*0.732=43.92rpm，加速度A=50*8.7deg/s^2，运行至P1=4095位置
  Position[0] = 4095;
  Position[1] = 4095;
  SyncWritePosEx(ID, 2, Position, Speed, ACC);
  HAL_Delay((4095-0)*1000/(60*50) + (60*50)*10/(50) + 50);//[(P1-P0)/(V*50)]*1000+[(V*50)/(A*100)]*1000 + 50(误差)

  //舵机(ID1/ID2)以最高速度V=60*0.732=43.92rpm，加速度A=50*8.7deg/s^2，运行至P0=0位置
  Position[0] = 0;
  Position[1] = 0;
  SyncWritePosEx(ID, 2, Position, Speed, ACC);
  HAL_Delay((4095-0)*1000/(60*50) + (60*50)*10/(50) + 50);//[(P1-P0)/(V*50)]*1000+[(V*50)/(A*100)]*1000 + 50(误差)
}
