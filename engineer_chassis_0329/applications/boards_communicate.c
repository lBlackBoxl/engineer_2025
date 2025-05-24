#include "FreeRTOS.h"
#include "main.h"
#include "boards_communicate.h"
#include "can_communicate.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "arm_control_task.h"
#include "usart.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern bool_t  clamp_flag;
extern uint8_t AJX_flag;
extern uint8_t error_flag;

uint8_t USART6_TX_Buffer[20];

#define get_board_communicate_data_0(ptr, data)                     			\
{                                                                   			\
    (ptr)->target_position[0] = (uint16_t)((data)[0] << 8 | (data)[1]);  	\
    (ptr)->target_position[1] = (uint16_t)((data)[2] << 8 | (data)[3]);   \
    (ptr)->target_position[2] = (uint16_t)((data)[4] << 8 | (data)[5]);   \
    (ptr)->target_position[3] = (uint16_t)((data)[6] << 8 | (data)[7]);  	\
}

#define get_board_communicate_data_1(ptr, data)                     			\
{                                                                   			\
    (ptr)->target_position[4] = (uint16_t)((data)[0] << 8 | (data)[1]);  	\
    (ptr)->target_position[5] = (uint16_t)((data)[2] << 8 | (data)[3]);   \
}		

static CAN_TxHeaderTypeDef  board_communicate_can1_tx_message;
static uint8_t 							board_communicate_can1_send_data[8];
static void TX_init(void);

Tx_Message_t tx_message_mp;

void board_communicate_task(void const *pvParameters)
{
		vTaskDelay(500);
		uint32_t mode_wake_time = osKernelSysTick();
		while(1)
		{
			TX_init();
			CAN_board_communicate_can_0(tx_message_mp.target_position);
			osDelay(1);
			CAN_board_communicate_can_1(tx_message_mp.target_position, tx_message_mp.mode, tx_message_mp.suker_key_flag);
			osDelay(1);
				
			HAL_UART_Transmit_DMA(&huart6,USART6_TX_Buffer,sizeof(USART6_TX_Buffer));
			osDelay(2);
		}
}

void TX_init(void)
{
	tx_message_mp.mode = chassis.chassis_mode<<8 | chassis.arm_mode;
	tx_message_mp.target_position[0] = arm_control.motor_1_position;
	tx_message_mp.target_position[1] = arm_control.motor_2_position;
	tx_message_mp.target_position[2] = arm_control.motor_3_position;
	tx_message_mp.target_position[3] = arm_control.motor_4_position;
	tx_message_mp.target_position[4] = arm_control.motor_5_position;
	tx_message_mp.target_position[5] = arm_control.motor_6_position;
	tx_message_mp.suker_key_flag = chassis.suker_key_flag;
	
	USART6_TX_Buffer[0] = 'i';
	USART6_TX_Buffer[1] = chassis.chassis_mode;
	USART6_TX_Buffer[2] = chassis.arm_mode;
	USART6_TX_Buffer[3] = chassis.move_mode;
	USART6_TX_Buffer[4] = suker_key_flag;
	USART6_TX_Buffer[5] = HAL_GPIO_ReadPin(Ore_1_GPIO_Port,Ore_1_Pin);
	USART6_TX_Buffer[6] = HAL_GPIO_ReadPin(Ore_2_GPIO_Port,Ore_2_Pin);
	USART6_TX_Buffer[7] = clamp_flag;
	USART6_TX_Buffer[8] = AJX_flag;
	USART6_TX_Buffer[9] = arm_restart_flag;
	USART6_TX_Buffer[10] = error_flag;
	USART6_TX_Buffer[19] = 'e';
}

void CAN_board_communicate_can_0(fp32 board_position_message[6])
{
		uint16_t board_position_message_tmp[6];
	  board_position_message_tmp[0] = float_to_uint(board_position_message[0], -50.0f, 50.0f, 16);
    board_position_message_tmp[1] = float_to_uint(board_position_message[1], -6.2831852f, 6.2831852f, 16);
    board_position_message_tmp[2] = float_to_uint(board_position_message[2], -6.2831852f, 6.2831852f, 16);
    board_position_message_tmp[3] = float_to_uint(board_position_message[3], -6.2831852f, 6.2831852f, 16);
		uint32_t send_mail_box;
    board_communicate_can1_tx_message.StdId = CAN_COMMUNICATE_TX_ID_0;
    board_communicate_can1_tx_message.IDE = CAN_ID_STD;
    board_communicate_can1_tx_message.RTR = CAN_RTR_DATA;
    board_communicate_can1_tx_message.DLC = 0x08;
		board_communicate_can1_send_data[0] = (board_position_message_tmp[0] >> 8);
    board_communicate_can1_send_data[1] = board_position_message_tmp[0];
		board_communicate_can1_send_data[2] = (board_position_message_tmp[1] >> 8);
    board_communicate_can1_send_data[3] = board_position_message_tmp[1];		
		board_communicate_can1_send_data[4] = (board_position_message_tmp[2] >> 8);
		board_communicate_can1_send_data[5] = board_position_message_tmp[2];		
		board_communicate_can1_send_data[6] = (board_position_message_tmp[3] >> 8);
    board_communicate_can1_send_data[7] = board_position_message_tmp[3];

    HAL_CAN_AddTxMessage(&BROADS_CAN, &board_communicate_can1_tx_message, board_communicate_can1_send_data, &send_mail_box);
}

void CAN_board_communicate_can_1(fp32 board_position_message[6],uint16_t mode, uint16_t suker_key_falg)
{
		uint16_t board_position_message_tmp[6];
	  board_position_message_tmp[4] = float_to_uint(board_position_message[4], -6.2831852f, 6.2831852f, 16);
    board_position_message_tmp[5] = float_to_uint(board_position_message[5], -6.2831852f, 6.2831852f, 16);
		uint32_t send_mail_box;
    board_communicate_can1_tx_message.StdId = CAN_COMMUNICATE_TX_ID_1;
    board_communicate_can1_tx_message.IDE = CAN_ID_STD;
    board_communicate_can1_tx_message.RTR = CAN_RTR_DATA;
    board_communicate_can1_tx_message.DLC = 0x08;
		board_communicate_can1_send_data[0] = (board_position_message_tmp[4] >> 8);
    board_communicate_can1_send_data[1] = board_position_message_tmp[4];
		board_communicate_can1_send_data[2] = (board_position_message_tmp[5] >> 8);
    board_communicate_can1_send_data[3] = board_position_message_tmp[5];
		if(arm_restart_flag == 0)
		{
			board_communicate_can1_send_data[4] = mode >> 8;
			board_communicate_can1_send_data[5] = mode;
		}
		else if(arm_restart_flag == 1)
		{
			board_communicate_can1_send_data[4] = 0;
			board_communicate_can1_send_data[5] = 0;
		}
		board_communicate_can1_send_data[6] = (suker_key_flag << 8);
		board_communicate_can1_send_data[7] = chassis.move_mode;
 
    HAL_CAN_AddTxMessage(&BROADS_CAN, &board_communicate_can1_tx_message, board_communicate_can1_send_data, &send_mail_box);
}
