#include "main.h"
#include <../../SCSLib/SCServo.h>
void setup(void)
{
	setEnd(1);//SCSCL舵机为大端存储结构
}

void examples(void)
{
	unLockEpromEx(1);//打开EPROM保存功能
  writeByte(1, SCSCL_ID, 2);//ID
  writeWord(2, SCSCL_MIN_ANGLE_LIMIT_L, 20);
	writeWord(2, SCSCL_MAX_ANGLE_LIMIT_L, 1000);
	LockEpromEx(2);//关闭EPROM保存功能
	while(1){}
}
