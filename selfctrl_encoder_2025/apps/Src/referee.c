#include "referee.h"
#include "protocol.h"
#include "bsp_uart.h"
#include "fifo.h"

tFrame referee_tx_msg;
unpack_data_t referee_unpack_obj;


static inline void referee_tx_init(tFrame* send_buf);
static inline void referee_tx_data_fill(tFrame* send_buf, fp32 motor_data[7]);
static inline uint8_t referee_tx_transmit(tFrame* send_buf);


uint8_t referee_tx_send(fp32 motor_data[7]){
		referee_tx_init(&referee_tx_msg);
		referee_tx_data_fill(&referee_tx_msg, motor_data);
		return referee_tx_transmit(&referee_tx_msg);
}

static void referee_tx_init(tFrame* send_buf)
{
	  send_buf->header.sof = 0xA5;
    send_buf->header.dataLenth = 30; // 4 * 7 + 1 * 2 = 30
    send_buf->header.seq = 0;
		send_buf->cmd_id = 0x0302;
		append_crc8_check_sum(&send_buf->header.sof, sizeof(tFrameHeader));
}

static void referee_tx_data_fill(tFrame* send_buf, fp32 motor_data[6]){
		static uint8_t i;
		static uint16_t time_stamp = 0x1111;
		for(i=0;i<6;i++){
				send_buf->data[i] = motor_data[i];
		}
		//adding time stamps
		send_buf->time_stamp = time_stamp++;
}

static uint8_t referee_tx_transmit(tFrame* send_buf){
		static uint8_t tx_buf[39];
		append_crc16_check_sum_referee(&send_buf->header.sof, sizeof(tFrame));
		memcpy(tx_buf, send_buf, sizeof(tx_buf));
		return HAL_UART_Transmit_DMA(&REFEREE_HUART, tx_buf, sizeof(tx_buf));;
}


//单字节解包
void referee_unpack_fifo_data(void)
{
	uint8_t byte = 0;
	uint8_t sof = 0xA5;
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
					if (verify_crc16_check_sum_referee(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
					{
						//add referee data soving function here
						//referee_data_solve(p_obj->protocol_packet);a
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


