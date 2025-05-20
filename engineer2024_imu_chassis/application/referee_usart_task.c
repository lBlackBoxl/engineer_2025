#include "referee_usart_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"
//裁判系统数据解包
static void referee_unpack_fifo_data(void);
extern UART_HandleTypeDef huart1;
uint8_t usart1_buf[2][USART_RX_BUF_LENGHT];
uint16_t cmd_time;
uint16_t dead_time; 
uint8_t dead_flag;
fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;
//裁判系统任务
void referee_usart_task(void const * argument)
{
    init_referee_struct_data();
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    usart1_init(usart1_buf[0], usart1_buf[1], USART_RX_BUF_LENGHT);
    while(1)
    {
		referee_unpack_fifo_data();
		//向电容发送数据
//		CAN_cmd_cap();
		dead_time++;
		if(dead_time == 50)
			dead_flag = 1;
		osDelay(10);
    }
}
//单字节解包
void referee_unpack_fifo_data(void)
{
	uint8_t byte = 0;
	uint8_t sof = HEADER_SOF;
	unpack_data_t *p_obj = &referee_unpack_obj;
	while ( fifo_s_used(&referee_fifo) )
	{
		byte = fifo_s_get(&referee_fifo);
		switch(p_obj->unpack_step)
		{
			case STEP_HEADER_SOF:
			{
				if(byte == sof)
				{
					p_obj->unpack_step = STEP_LENGTH_LOW;
					p_obj->protocol_packet[p_obj->index++] = byte;
				}
				else
				{  
					p_obj->index = 0;
				}
			}
			break;
			case STEP_LENGTH_LOW:
			{
				p_obj->data_len = byte;
				p_obj->protocol_packet[p_obj->index++] = byte;
				p_obj->unpack_step = STEP_LENGTH_HIGH;
			}
			break;
			case STEP_LENGTH_HIGH:
			{
				p_obj->data_len |= (byte << 8);
				p_obj->protocol_packet[p_obj->index++] = byte;
				if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
				{
					p_obj->unpack_step = STEP_FRAME_SEQ;
				}
				else
				{
					p_obj->unpack_step = STEP_HEADER_SOF;
					p_obj->index = 0;
				}
			}
			break;
			case STEP_FRAME_SEQ:
			{
				p_obj->protocol_packet[p_obj->index++] = byte;
				p_obj->unpack_step = STEP_HEADER_CRC8;
			}
			break;
			case STEP_HEADER_CRC8:
			{
				p_obj->protocol_packet[p_obj->index++] = byte;
				if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
				{
					if ( verify_crc8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
					{
						p_obj->unpack_step = STEP_DATA_CRC16;
					}
					else
					{
						p_obj->unpack_step = STEP_HEADER_SOF;
						p_obj->index = 0;
					}
				}
			}
			break;  
			case STEP_DATA_CRC16:
			{
				if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
				{
					p_obj->protocol_packet[p_obj->index++] = byte;  
				}
				if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
				{
					p_obj->unpack_step = STEP_HEADER_SOF;
					p_obj->index = 0;
					if ( verify_crc16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
					{
						referee_data_solve(p_obj->protocol_packet);
					}
			}
			}break;
			default:
			{
				p_obj->unpack_step = STEP_HEADER_SOF;
				p_obj->index = 0;
			}
			break;
		}
	}
}
void USART1_IRQHandler(void)
{
    static volatile uint8_t res;
	dead_flag = 0;
    if(USART1->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
        static uint16_t this_time_rx_len = 0;
        if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, USART_RX_BUF_LENGHT);
            huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart1.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart1_buf[0], this_time_rx_len);
        }
        else
        {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, USART_RX_BUF_LENGHT);
            huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart1.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart1_buf[1], this_time_rx_len);
        }
    }
  HAL_UART_IRQHandler(&huart1);
}
