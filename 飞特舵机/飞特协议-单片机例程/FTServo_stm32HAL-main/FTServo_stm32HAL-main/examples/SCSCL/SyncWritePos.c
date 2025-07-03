#include "main.h"
#include <../../SCSLib/SCServo.h>

uint8_t ID[2];
uint16_t Position[2];
uint16_t Speed[2];

void setup(void)
{
	setEnd(1);//SCSCL���Ϊ��˴洢�ṹ
  ID[0] = 1;//���ID1
  ID[1] = 2;//���ID2
  Speed[0] = 1500;//����ٶ�V=1500*0.059=88.5rpm
  Speed[1] = 1500;//����ٶ�V=1500*0.059=88.5rpm
}

void examples(void)
{
	//���(ID1/ID2)������ٶ�V=1500*0.059=88.5rpm��������P1=1000λ��
  Position[0] = 1000;
  Position[1] = 1000;
  SyncWritePos(ID, 2, Position, 0, Speed);
	HAL_Delay((1000-20)*1000/(1500) + 100);//[(P1-P0)/V]*1000 + 100
	
  //���(ID1/ID2)������ٶ�V=1500*0.059=88.5rpm��������P0=20λ��
  Position[0] = 20;
  Position[1] = 20;
  SyncWritePos(ID, 2, Position, 0, Speed);
	HAL_Delay((1000-20)*1000/(1500) + 100);//[(P1-P0)/V]*1000 + 100
}
