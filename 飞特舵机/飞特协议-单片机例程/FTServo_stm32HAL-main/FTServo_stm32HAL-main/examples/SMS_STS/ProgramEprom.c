#include "main.h"
#include <../../SCSLib/SCServo.h>
void setup(void)
{
	setEnd(0);//SMS_STS舵机为小端存储结构
}

void examples(void)
{
	unLockEprom(1);//打开EPROM保存功能
  writeByte(1, SCSCL_ID, 2);//ID
	LockEprom(2);//关闭EPROM保存功能
	while(1){}
}
