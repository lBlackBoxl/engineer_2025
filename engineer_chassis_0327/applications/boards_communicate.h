#ifndef BOARDS_COMMUNICATE_H
#define BOARDS_COMMUNICATE_H

#include "main.h"
#include "struct_typedef.h"

#define COMMUNICATE_HEADER1  0xFE
#define COMMUNICATE_HEADER2  0xFE
#define COMMUNICATE_TAIL 0xA3
#define COMMUNICATE_TX_LEN 26
#define COMMUNICATE_RX_LEN 17


//typedef struct 
//{  
//	//以 4个字节一组排列 ，不然编译器会凑整
//	 uint8_t head[2];
//}Head_t;

typedef struct
{
	//以 4个字节一组排列 ，不然编译器会凑整
	 fp32 motor_position[6];
}Rx_Message_t;

typedef struct
{
	//以 4个字节一组排列 ，不然编译器会凑整
//	 Head_t Head;
	 uint16_t mode;
	 uint16_t suker_key_flag;
	 fp32 target_position[6];
	 //fp32 max_speed[5];
}Tx_Message_t;

extern fp32 rx_motor_position[5];  //当前电机反馈角度

extern Tx_Message_t tx_message_mp;

extern void CAN_board_communicate_can_0(fp32 board_position_message[6]);

void CAN_board_communicate_can_1(fp32 board_position_message[6],uint16_t mode, uint16_t suker_key_falg);

extern void board_communicate_task(void const *pvParameters);

#endif
