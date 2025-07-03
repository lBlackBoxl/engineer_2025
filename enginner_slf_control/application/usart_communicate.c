#include "usart_communicate.h"
#include "usart.h"
#include "bsp_usart.h"
#include "string.h"
#include "cmsis_os.h"
#include "self_control_task.h"
#include "struct_typedef.h"

uint8_t seq = 0;
send_data send_data_self_control;
uint8_t tx_buffer_self_control[39];
extern float Present_Position[6];
//tx_self_control_message tx_self_control;

static void tx_init(void);

void usart_communicate_task(void const * argument)
{
	osDelay(2000);
	tx_init();
	usart6_idle_init();

	while(1)
	{
		osDelay(40);
		send_data_self_control.header.seq = seq++;
		
		for(int i = 0; i < 6; i++)
		{
				send_data_self_control.data[i] = Present_Position[i];
		}
		
		append_crc8_check_sum(&send_data_self_control.header.sof, 5);
		append_crc16_check_sum(&send_data_self_control.header.sof, 39);
		memcpy(tx_buffer_self_control,&send_data_self_control, 39);
		HAL_UART_Transmit_DMA(&huart6,tx_buffer_self_control,39);
		
	}
}

void tx_init()
{
	  send_data_self_control.header.sof = 0xA5;
    send_data_self_control.header.dataLenth = 30; // 4 * 7 + 1 * 2 = 30
    send_data_self_control.header.seq = 0;
		send_data_self_control.cmd_id = 0x0302;
}
