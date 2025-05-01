#ifndef USART_COMMUNICATE_H
#define USART_COMMUNICATE_H

#include "stdint.h"
#include "protocol.h"

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

#endif
