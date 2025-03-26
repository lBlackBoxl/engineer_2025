#ifndef BOARDS_COMMUNICATE_TASK_H
#define BOARDS_COMMUNICATE_TASK_H

#include "struct_typedef.h"

typedef struct 
{  
	//以 4个字节一组排列 ，不然编译器会凑整
	 fp32 motor_position[6];
}Tx_Message_t;

typedef struct 
{  
	//以 4个字节一组排列 ，不然编译器会凑整
	 uint16_t mode;
	 fp32 target_position[6];
}Rx_Message_t;

extern void A_communicate_task(void const * argument);
extern Rx_Message_t rx_message_mp; 
extern Tx_Message_t tx_message_mp;
extern uint16_t last_mode;

#endif
