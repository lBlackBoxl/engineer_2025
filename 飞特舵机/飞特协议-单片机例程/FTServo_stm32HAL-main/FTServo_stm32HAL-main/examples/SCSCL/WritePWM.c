#include "main.h"
#include <../../SCSLib/SCServo.h>

void setup(void)
{
	setEnd(1);//SCSCL���Ϊ��˴洢�ṹ
	PWMMode(1);//����л���PWM�������ٶ�ģʽ
}

void examples(void)
{
  //���(ID1)�����50%Ť��������ת
	WritePWM(1, 500);
	HAL_Delay(2000);
	
	//���(ID1)ֹͣ��ת
	WritePWM(1, 0);
	HAL_Delay(2000);
	
  //���(ID1)�����50%Ť�ط�����ת
	WritePWM(1, -500);
	HAL_Delay(2000);
  
  //���(ID1)ֹͣ��ת
	WritePWM(1,0);
	HAL_Delay(2000);
}
