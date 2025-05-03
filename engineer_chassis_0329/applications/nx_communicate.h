#ifndef NX_COMMUNICATE_H
#define NX_COMMUNICATE_H

#include "main.h"
#include "struct_typedef.h"

#define SOF_ADDR 	 0			//帧头多项式字节偏移量

#define FRAME_HEADER 		0x69//帧头多项式
#define LEN_FRAME_HEADER 	1	//帧头长度
#define	LEN_TX_DATA 		24	//发送数据段长度
#define	LEN_RX_DATA 		24	//接收数据段长度
#define	LEN_FRAME_TAILER 	2	//帧尾CRC16
#define	LEN_TX_PACKET		28	//发送包整包长度
#define	LEN_RX_PACKET		2	//接收包整包长度

#define USART7_MAX_RECV_LEN 100

void nx_communicate_task(void const *pvParameters);

typedef __packed struct
{
	uint8_t  	sof;			// 帧头多项式
} nx_frame_header_t;
//帧尾结构体
typedef __packed struct 
{
	uint8_t crc16;				// CRC16校验码
} nx_frame_tailer_t;

//发送数据结构体
typedef __packed struct 
{
	fp32 motor1_position;
	fp32 motor2_position;
	fp32 motor3_position;
	fp32 motor4_position;
	fp32 motor5_position;
	fp32 motor6_position;
}nx_tx_data_t;

//接受数据结构体
typedef __packed struct 
{
	fp32 motor1_position;
	fp32 motor2_position;
	fp32 motor3_position;
	fp32 motor4_position;
	fp32 motor5_position;
	fp32 motor6_position;
}nx_rx_data_t;

//发送包结构体
typedef __packed struct 
{
	nx_frame_header_t 		frame_header;	
	nx_tx_data_t	  		tx_data;	
	uint8_t					robot_id;
	uint8_t         arm_move_flag;
	nx_frame_tailer_t 		frame_tailer;	
} nx_send_msg_t;

//接受包结构体
typedef __packed struct 
{
	nx_frame_header_t	 	frame_header;	
	nx_rx_data_t	  		rx_data;	
	nx_frame_tailer_t 		frame_tailer;	
} nx_receive_msg_t;

extern nx_send_msg_t pc_send_msg;
extern nx_receive_msg_t pc_receive_msg;
extern float nx_allowance[6];
#endif
