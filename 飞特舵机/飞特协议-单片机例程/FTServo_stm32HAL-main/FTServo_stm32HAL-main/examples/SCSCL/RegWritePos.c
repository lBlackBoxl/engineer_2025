#include "main.h"
#include <../../SCSLib/SCServo.h>

void setup(void)
{
	setEnd(1);//SCSCL���Ϊ��˴洢�ṹ
}

void examples(void)
{
	//���(ID1/ID2)������ٶ�V=1500*0.059=88.5rpm��������P1=1000λ��
  RegWritePos(1, 1000, 0, 1500);
  RegWritePos(2, 1000, 0, 1500);
  RegWriteAction();
  HAL_Delay((1000-20)*1000/(1500) + 100);//[(P1-P0)/V]*1000 + 100
	
  //���(ID1/ID2)������ٶ�V=1500*0.059=88.5rpm��������P0=20λ��
  RegWritePos(1, 20, 0, 1500);
  RegWritePos(2, 20, 0, 1500);
  RegWriteAction();
  HAL_Delay((1000-20)*1000/(1500) + 100);//[(P1-P0)/V]*1000 + 100
}
