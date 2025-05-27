#ifndef ENCODER_TASK_H
#define ENCODER_TASK_H

#include "bsp_uart.h"

typedef enum{
	UNUPDATE	= 0x00,
	UPDATING 	= 0x01,
	UPDATED		= 0x02,
	
	ERROR_TX	= 0x80,
	ERROR_RX	=	0x40,
}J_Update_Status_e;

typedef struct{
	fp32 ratio;
	fp32 offset;
	fp32 min;
	fp32 max;
}Joint_trans_t;

typedef struct{
	J_Update_Status_e	status;
	fp32 joint[6];
	int16_t cycle[6];
	Joint_trans_t limit[6];
}J_Data_t;

extern J_Data_t* get_arm_ptr(void);

#endif
