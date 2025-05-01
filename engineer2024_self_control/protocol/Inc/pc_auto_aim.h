#ifndef __PC_INFO_TASK_H
#define __PC_INFO_TASK_H
#include "stm32f4xx.h"
#include "stdint.h"
#include "arm_math.h"

#define SOF_ADDR 	 0			//帧头多项式字节偏移量
#define FRAME_HEADER 		0xA5//帧头多项式
#define LEN_FRAME_HEADER 	2	//帧头长度
#define	LEN_TX_DATA 		14	//发送数据段长度
#define	LEN_RX_DATA 		18	//接收数据段长度
#define	LEN_FRAME_TAILER 	2	//帧尾CRC16
#define	LEN_TX_PACKET		18	//发送包整包长度23
#define	LEN_RX_PACKET		21	//接收包整包长度


//帧头结构体
typedef __packed struct
{
    uint8_t sof;      //帧头多项式
    uint8_t crc8;     //crc8校验码
}frame_header_t;
//帧尾结构体
typedef __packed struct 
{
    uint16_t crc16;   //crc16校验码
}frame_tailer_t;
//发送自瞄数据
typedef __packed struct 
{
    float32_t shoot_speed;
    float32_t curr_yaw;
    float32_t curr_pitch;
	  uint8_t   state;  //0是自瞄 1是running
    uint8_t   enemy_color;
	uint8_t   hit[8];
	uint8_t   is_balance[3];
}auto_aim_tx_data_t;
//接收自瞄数据
typedef __packed struct 
{
    float32_t shoot_yaw;  //偏转角
    float32_t shoot_pitch;
    uint8_t fire;   //1为开火
}auto_aim_rx_data_t;
//自瞄发送包结构体
typedef __packed struct 
{
    frame_header_t frame_header;
    auto_aim_tx_data_t tx_data;
    frame_tailer_t frame_tailer;
}auto_aim_send_msg_t;
//自瞄接收包结构体
typedef __packed struct 
{
    frame_header_t frame_header;
    auto_aim_rx_data_t rx_data;
    frame_tailer_t framer_tailer;
}auto_aim_receive_msg_t;
extern float before_yaw;
extern float before_pitch;
extern auto_aim_send_msg_t auto_aim_send_msg;
extern auto_aim_receive_msg_t auto_aim_receive_msg;
extern void pc_auto_aim_task(void const *argu);

#endif

