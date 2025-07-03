#include "main.h"
#include "STS_app.h"
#include "STS_drive.h"
#include "cmsis_os.h"
#include "bsp_adc.h"
#include "SCServo.h"
#include "SMS_STS.h"

void STS_Init();
extern int WheelMode(uint8_t ID);//恒速模式
extern int WriteSpe(uint8_t ID, int16_t Speed, uint8_t ACC);//恒速模式控制指令
extern void STS_CMD_POS_ADJ(uint8_t STS_ID, uint16_t target_position);

void STS_Init()
{
		setEnd(0);//SMS_STS舵机为大端存储结构
	
		for(int i = 1; i < 5; i++)
		{
			//舵机切换至电机恒速模式
			WheelMode(i);
			WriteSpe(i, 0, 0);
			
			//禁止舵机力输出
			writeWord(i, 0x28, 0);

//			position_allowance[i] = readWord(i, 0x1F);
//			position_allowance[i] = position_allowance[i] + (4096 - ReadPos(i));
//			writeWord(i, 0x1F, position_allowance[i]);
//			STS_CMD_POS_ADJ(i, 0);
		}

}
