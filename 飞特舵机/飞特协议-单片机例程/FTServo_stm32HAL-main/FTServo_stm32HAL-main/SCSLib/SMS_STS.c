/*
 * SMS_STS.c
 * 飞特SMS/STS系列串行舵机应用层程序
 * 日期: 2025.3.3
 * 作者: 
 */

#include <string.h>
#include "INST.h"
#include "SCS.h"
#include "SMS_STS.h"

int WritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC)
{
	uint8_t bBuf[7];
	if(Position<0){
		Position = -Position;
		Position |= (1<<15);
	}

	bBuf[0] = ACC;
	Host2SCS(bBuf+1, bBuf+2, Position);
	Host2SCS(bBuf+3, bBuf+4, 0);
	Host2SCS(bBuf+5, bBuf+6, Speed);
	
	return genWrite(ID, SMS_STS_ACC, bBuf, 7);
}

int RegWritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC)
{
	uint8_t bBuf[7];
	if(Position<0){
		Position = -Position;
		Position |= (1<<15);
	}

	bBuf[0] = ACC;
	Host2SCS(bBuf+1, bBuf+2, Position);
	Host2SCS(bBuf+3, bBuf+4, 0);
	Host2SCS(bBuf+5, bBuf+6, Speed);
	
	return regWrite(ID, SMS_STS_ACC, bBuf, 7);
}

void SyncWritePosEx(uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Speed[], uint8_t ACC[])
{
	uint8_t offbuf[32*7];
	uint8_t i;
	uint16_t V;
  for(i = 0; i<IDN; i++){
		if(Position[i]<0){
			Position[i] = -Position[i];
			Position[i] |= (1<<15);
		}

		if(Speed){
			V = Speed[i];
		}else{
			V = 0;
		}
		if(ACC){
			offbuf[i*7] = ACC[i];
		}else{
			offbuf[i*7] = 0;
		}
		Host2SCS(offbuf+i*7+1, offbuf+i*7+2, Position[i]);
    Host2SCS(offbuf+i*7+3, offbuf+i*7+4, 0);
    Host2SCS(offbuf+i*7+5, offbuf+i*7+6, V);
	}
  syncWrite(ID, IDN, SMS_STS_ACC, offbuf, 7);
}

int WheelMode(uint8_t ID)
{
	return writeByte(ID, SMS_STS_MODE, 1);		
}

int WriteSpe(uint8_t ID, int16_t Speed, uint8_t ACC)
{
	uint8_t bBuf[2];
	if(Speed<0){
		Speed = -Speed;
		Speed |= (1<<15);
	}
	bBuf[0] = ACC;
	genWrite(ID, SMS_STS_ACC, bBuf, 1);
	
	Host2SCS(bBuf+0, bBuf+1, Speed);

	genWrite(ID, SMS_STS_GOAL_SPEED_L, bBuf, 2);
	return 1;
}

int CalibrationOfs(uint8_t ID)
{
	return writeByte(ID, SMS_STS_TORQUE_ENABLE, 128);
}

int unLockEpromEx(uint8_t ID)
{
	return writeByte(ID, SMS_STS_LOCK, 0);
}

int LockEpromEx(uint8_t ID)
{
	return writeByte(ID, SMS_STS_LOCK, 1);
}
