#ifndef REFEREE_USART_TASK_H
#define REFEREE_USART_TASK_H

#include "main.h"
#include "referee.h"

#define USART_RX_BUF_LENGHT     512
#define REFEREE_FIFO_BUF_LENGTH 1024

typedef __packed struct
{
		tFrameHeader                           FrameHead;                 //÷°Õ∑
		uint16_t                               CmdId;                     //√¸¡Ó¬Î 
		ext_arm_psoe_t    arm_pose;
		uint16_t                               CRC16;
}ARM_T;

extern void referee_usart_task(void const * argument);
extern void USART6_IRQHandler(void);
#endif
