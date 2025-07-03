#include "main.h"
#include <../../SCSLib/SCServo.h>

void setup(void)
{
	setEnd(1);//SCSCL舵机为大端存储结构
}

void examples(void)
{
	//舵机(ID1/ID2)以最高速度V=1500*0.059=88.5rpm，运行至P1=1000位置
  RegWritePos(1, 1000, 0, 1500);
  RegWritePos(2, 1000, 0, 1500);
  RegWriteAction();
  HAL_Delay((1000-20)*1000/(1500) + 100);//[(P1-P0)/V]*1000 + 100
	
  //舵机(ID1/ID2)以最高速度V=1500*0.059=88.5rpm，运行至P0=20位置
  RegWritePos(1, 20, 0, 1500);
  RegWritePos(2, 20, 0, 1500);
  RegWriteAction();
  HAL_Delay((1000-20)*1000/(1500) + 100);//[(P1-P0)/V]*1000 + 100
}
