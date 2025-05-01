#ifndef PC_TASK_H
#define PC_TASK_H
#include "struct_typedef.h"
#include "usart.h"
#define SOF_ADDR 	 0			//帧头多项式字节偏移量

#define FRAME_HEADER 		0xA5//帧头多项式
#define LEN_FRAME_HEADER 	2	//帧头长度
#define	LEN_TX_DATA 		11	//发送数据段长度
#define	LEN_RX_DATA 		18	//接收数据段长度
#define	LEN_FRAME_TAILER 	2	//帧尾CRC16
#define	LEN_TX_PACKET		15	//发送包整包长度
#define	LEN_RX_PACKET	  13	//接收包整包长度

//最大缓冲字节数
#define USART1_MAX_RECV_LEN 100

#define PC_RX_BUF_NUM   26u
#define PC_RX_BUF_LENGTH 13u
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
//发送数据结构体
typedef __packed struct 
{
	float curr_yaw;         	//当前云台yaw角度
	float curr_pitch;       	//当前云台pitch角
	uint8_t state;          	//当前状态，自瞄-大符-小符
	uint8_t pc_state;        //自瞄模式
	uint8_t enemy_color;    	//敌方颜色
}tx_data_t;
//接受数据结构体
typedef __packed struct 
{
	uint8_t fire;            	//发射指令
	float shoot_yaw;       		//最终偏转角
	float shoot_pitch;
}rx_data_t;
//发送包结构体
typedef __packed struct 
{
	frame_header_t 		frame_header;	
	tx_data_t	  		   tx_data;	
	frame_tailer_t 		frame_tailer;	
} send_msg_t;
//接受包结构体
typedef __packed struct 
{
	frame_header_t	 	frame_header;	
	rx_data_t	  		  rx_data;	
	frame_tailer_t 		frame_tailer;	
} receive_msg_t;

extern float before_yaw;
extern float before_pitch;
extern receive_msg_t pc_receive_msg;
extern void pc_task(void const *argu);
extern void USART6_RX_IRQHandler(void);
const rx_data_t *get_pc_rx_point(void);
#endif
