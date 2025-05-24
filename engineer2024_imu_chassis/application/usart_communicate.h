#ifndef USART_COMMUNICATE_H
#define USART_COMMUNICATE_H

#include "stdint.h"
#include "protocol.h"

#define USART6_MAX_RECV_LEN 50

//帧头结构体
typedef __packed struct
{
	uint8_t  	sof;			// 帧头多项式
	uint8_t  	crc8;			// CRC8校验码
} frame_header_t;
//帧尾结构体
typedef __packed struct 
{
	uint16_t crc16;				// CRC16校验码
} frame_tailer_t;

typedef  __packed struct
{
    float yaw_angle;
}tx_data_t;

//发送包结构体
typedef __packed struct 
{
	frame_header_t 		frame_header;	
	tx_data_t	  		tx_data;	
	frame_tailer_t 		frame_tailer;	
} send_msg_t;

extern send_msg_t tx_imu_data;
void usart_communicate_task(void const * argument);

extern uint8_t chassis_mode;
extern uint8_t arm_mode;
extern uint8_t move_mode;
extern uint8_t suker_key_flag;
extern uint8_t Ore_1_flag;
extern uint8_t Ore_2_flag;
extern uint8_t clamp_flag;
extern uint8_t AJX_flag;
extern uint8_t arm_restart_flag;
extern uint8_t arm_error_flag;
#endif
