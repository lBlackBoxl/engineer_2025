#include "main.h"
#include <../../SCSLib/SCServo.h>

void setup(void)
{
	setEnd(1);//SCSCL舵机为大端存储结构
	PWMMode(1);//舵机切换到PWM开环调速度模式
}

void examples(void)
{
  //舵机(ID1)以最大50%扭矩正向旋转
	WritePWM(1, 500);
	HAL_Delay(2000);
	
	//舵机(ID1)停止旋转
	WritePWM(1, 0);
	HAL_Delay(2000);
	
  //舵机(ID1)以最大50%扭矩反向旋转
	WritePWM(1, -500);
	HAL_Delay(2000);
  
  //舵机(ID1)停止旋转
	WritePWM(1,0);
	HAL_Delay(2000);
}
