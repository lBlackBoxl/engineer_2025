#include "main.h"
#include <../../SCSLib/SCServo.h>

void setup(void)
{
	setEnd(1);//SCSCL���Ϊ��˴洢�ṹ
}

void examples(void)
{
	//���(�㲥)������ٶ�V=1500*0.059=88.5rpm��������P1=1000λ��
	WritePos(0xfe, 1000, 0, 1500);
	HAL_Delay((1000-20)*1000/(1500) + 100);//[(P1-P0)/V]*1000 + 100
	
	//���(�㲥)������ٶ�V=1500*0.059=88.5rpm��������P0=20λ��
	WritePos(0xfe, 20, 0, 1500);//���(ID1),������ٶ�V=1500��/��,������P0=20
	HAL_Delay((1000-20)*1000/(1500) + 100);//[(P1-P0)/V]*1000 + 100
}
