#include "main.h"
#include <../../SCSLib/SCServo.h>

uint8_t ID[2];
uint16_t Position[2];
uint16_t Speed[2];

void setup(void)
{
	setEnd(1);//SCSCL舵机为大端存储结构
  ID[0] = 1;//舵机ID1
  ID[1] = 2;//舵机ID2
  Speed[0] = 1500;//最高速度V=1500*0.059=88.5rpm
  Speed[1] = 1500;//最高速度V=1500*0.059=88.5rpm
}

void examples(void)
{
	//舵机(ID1/ID2)以最高速度V=1500*0.059=88.5rpm，运行至P1=1000位置
  Position[0] = 1000;
  Position[1] = 1000;
  SyncWritePos(ID, 2, Position, 0, Speed);
	HAL_Delay((1000-20)*1000/(1500) + 100);//[(P1-P0)/V]*1000 + 100
	
  //舵机(ID1/ID2)以最高速度V=1500*0.059=88.5rpm，运行至P0=20位置
  Position[0] = 20;
  Position[1] = 20;
  SyncWritePos(ID, 2, Position, 0, Speed);
	HAL_Delay((1000-20)*1000/(1500) + 100);//[(P1-P0)/V]*1000 + 100
}
