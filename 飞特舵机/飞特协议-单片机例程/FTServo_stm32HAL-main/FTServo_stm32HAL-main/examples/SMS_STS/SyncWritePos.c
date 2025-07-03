#include "main.h"
#include <../../SCSLib/SCServo.h>

uint8_t ID[2];
int16_t Position[2];
uint16_t Speed[2];
uint8_t ACC[2];

void setup(void)
{
	setEnd(0);//SMS_STS���Ϊ��˴洢�ṹ
  ID[0] = 1;//���ID1
  ID[1] = 2;//���ID2
  Speed[0] = 60;//����ٶ�V=60*0.732=43.92rpm
  Speed[1] = 60;//����ٶ�V=60*0.732=43.92rpm
  ACC[0] = 50;//���ٶ�A=50*8.7deg/s^2
  ACC[1] = 50;//���ٶ�A=50*8.7deg/s^2
}

void examples(void)
{
  //���(ID1/ID2)������ٶ�V=60*0.732=43.92rpm�����ٶ�A=50*8.7deg/s^2��������P1=4095λ��
  Position[0] = 4095;
  Position[1] = 4095;
  SyncWritePosEx(ID, 2, Position, Speed, ACC);
  HAL_Delay((4095-0)*1000/(60*50) + (60*50)*10/(50) + 50);//[(P1-P0)/(V*50)]*1000+[(V*50)/(A*100)]*1000 + 50(���)

  //���(ID1/ID2)������ٶ�V=60*0.732=43.92rpm�����ٶ�A=50*8.7deg/s^2��������P0=0λ��
  Position[0] = 0;
  Position[1] = 0;
  SyncWritePosEx(ID, 2, Position, Speed, ACC);
  HAL_Delay((4095-0)*1000/(60*50) + (60*50)*10/(50) + 50);//[(P1-P0)/(V*50)]*1000+[(V*50)/(A*100)]*1000 + 50(���)
}
