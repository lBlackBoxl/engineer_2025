#include "referee_usart_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "can_communicate.h"
#include "bsp_usart.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"
#include "remote_control.h"
#include "bsp_rc.h"
#include "keyboard.h"
#include "bsp_dwt.h"
#include "bsp_rc.h"
#include "bsp_usart.h"

//裁判系统数据解包
static void referee_unpack_fifo_data(void);
extern UART_HandleTypeDef huart6;
uint8_t usart2_buf[2][USART_RX_BUF_LENGHT];
uint16_t cmd_time;
fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;
uint8_t Flag;
//遥控器控制变量
RC_ctrl_t rc_ctrl;
key_t last_s[5];

//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

//裁判系统任务
void referee_usart_task(void const * argument)
{
    init_referee_struct_data();
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    //usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);

    while(1)
    {
				referee_unpack_fifo_data();
				osDelay(2);
    }
}

//取正函数
static int16_t RC_abs(int16_t value);
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */

/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);


//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
	
void serial2_selfcontrol_rc_init(void)
{
//    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
		usart2_init(usart2_buf[0], usart2_buf[1], USART_RX_BUF_LENGHT);
}


/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

//判断遥控器数据是否出错，
uint8_t RC_data_is_error(void)
{
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
		if (rc_ctrl.header.frame[0] != RC_FRAME_1)
		{
			  goto error;
		}
		if (rc_ctrl.header.frame[1] != RC_FRAME_2)
		{
			  goto error;
		}
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_LEFT;
    rc_ctrl.rc.s[1] = RC_SW_LEFT;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    return 1;
}

void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}


//取正函数
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}

/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL) 
    {
        return;
    }
		
		rc_ctrl->header.frame[0] = sbus_buf[0];
		rc_ctrl->header.frame[1] = sbus_buf[1];
		
		if(!((rc_ctrl->header.frame[0] == 0xA9) && (rc_ctrl->header.frame[1] == 0x53)))
		{
			  return;
		}
		
		uint16_t key_information;
		key_information = (sbus_buf[17] | (sbus_buf[18] << 8));                 //键盘数据
		
    rc_ctrl->rc.ch[0] = (sbus_buf[2] | (sbus_buf[3] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[3] >> 3) | (sbus_buf[4] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 6) | (sbus_buf[5] << 2) |          //!< Channel 2
                         (sbus_buf[6] << 10)) &0x07ff;
    rc_ctrl->rc.ch[2] = ((sbus_buf[6] >> 1) | (sbus_buf[7] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[1] = ((sbus_buf[7] >> 4) & 0x0003);                   //挡位切换开关
    rc_ctrl->rc.s[0] = ((sbus_buf[7] >> 6) & 0x0001);                       //暂停按键
		rc_ctrl->rc.s[2] = ((sbus_buf[7] >> 7) & 0x0001);												//自定义按键（left）
		rc_ctrl->rc.s[3] = (sbus_buf[8] & 0x0001);														  //自定义按键（right）
		rc_ctrl->rc.ch[4] =((sbus_buf[8] >> 1) | (sbus_buf[9] << 7)) & 0x07ff;  //拨轮
		rc_ctrl->rc.s[4] = ((sbus_buf[9] >> 4) & 0x0001);                       //扳机
		
    rc_ctrl->mouse.x = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[12] | (sbus_buf[13] << 8);                  //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[14] | (sbus_buf[15] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[16] & 0x0001;                         //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_l = (sbus_buf[16] >> 2) & 0x0001;                  //!< Mouse Right Is Press ?
		rc_ctrl->mouse.press_l = (sbus_buf[16] >> 4) & 0x0001;								  //!< Mouse Mid Is Press ?
		
//    rc_ctrl->key.v = sbus_buf[17] | (sbus_buf[18] << 8);                    //!< KeyBoard value
    
		rc_ctrl->CRC16.crc_data = sbus_buf[19] | (sbus_buf[20] << 8);
		
    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
		
		rc_ctrl->key.W  = (key_information >> 0) & 1;  //按键W
		rc_ctrl->key.S  = (key_information >> 1) & 1;  //按键S
		rc_ctrl->key.A  = (key_information >> 2) & 1;  //按键A
		rc_ctrl->key.D  = (key_information >> 3) & 1;  //按键D
		rc_ctrl->key.SHIFT  = (key_information >> 4) & 1;  //按键SHIFT
		rc_ctrl->key.CTRL  = (key_information >> 5) & 1;  //按键CTRL
		rc_ctrl->key.Q  = (key_information >> 6) & 1;  //按键Q
		rc_ctrl->key.E  = (key_information >> 7) & 1;  //按键E
		rc_ctrl->key.R  = (key_information >> 8) & 1;  //按键R
		rc_ctrl->key.F  = (key_information >> 9) & 1;  //按键F
		rc_ctrl->key.G  = (key_information >> 10) & 1;  //按键G
		rc_ctrl->key.Z  = (key_information >> 11) & 1;  //按键Z
		rc_ctrl->key.X  = (key_information >> 12) & 1;  //按键X
		rc_ctrl->key.C  = (key_information >> 13) & 1;  //按键C
		rc_ctrl->key.V  = (key_information >> 14) & 1;  //按键V
		rc_ctrl->key.B  = (key_information >> 15) & 1;  //按键B
}

//单字节解包
void referee_unpack_fifo_data(void)
{
	uint8_t byte = 0;
	uint8_t sof = HEADER_SOF_SELF_CONTROL;
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
#if RC_FIFO_ENABLE
				else if(byte == HEADER_SOF_RC_L){
					p_obj->unpack_step = STEP_RC_SOF;
					p_obj->protocol_packet[p_obj->index++] = byte;	
				}
#endif
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
				if(p_obj->data_len == 30||p_obj->data_len == 28)
							Flag = 1;
						else
							Flag = 0;
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
					if (verify_crc16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
					{
						referee_data_solve(p_obj->protocol_packet);
					}
				}
			}break;
#if RC_FIFO_ENABLE
			case STEP_RC_SOF:
			{
				if(byte == HEADER_SOF_RC_H){
					p_obj->unpack_step = STEP_RC_DATA;
					p_obj->protocol_packet[p_obj->index++] = byte;
					p_obj->data_len = RC_DATA_LEN;
				}
				else{
					p_obj->unpack_step = STEP_HEADER_SOF;
					p_obj->index = 0;
				}
				break;
			}
			case STEP_RC_DATA:
			{
				if (p_obj->index < RC_FRAME_LEN)
				{
					p_obj->protocol_packet[p_obj->index++] = byte;  
				}
				if (p_obj->index >= RC_FRAME_LEN)
				{
					p_obj->unpack_step = STEP_HEADER_SOF;
					p_obj->index = 0;
					if(verify_crc16_check_sum(p_obj->protocol_packet, RC_FRAME_LEN)){
						//add RC data solving function here
						sbus_to_rc(p_obj->protocol_packet, &rc_ctrl);
					}
				}
				break;
			}
#endif //endif REMOTE_CONTROL_NEW_ENABLE
			default:
			{
				p_obj->unpack_step = STEP_HEADER_SOF;
				p_obj->index = 0;
			}
			break;
		}
	}
}


//串口中断
void USART2_IRQHandler(void)
{
	  static volatile uint8_t res;

    if(huart2.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart2);
    }
    else if(USART2->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart2);

        if ((hdma_usart2_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(&hdma_usart2_rx);

            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart2_rx.Instance->NDTR;

            hdma_usart2_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            hdma_usart2_rx.Instance->CR |= DMA_SxCR_CT;

            __HAL_DMA_ENABLE(&hdma_usart2_rx);

#if		!RC_FIFO_ENABLE
            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(usart2_buf[0], &rc_ctrl);
            }
		
						if(this_time_rx_len == SELF_CONTROL_FRAME_LENGTH)
            {	
								fifo_s_puts(&referee_fifo, (char*)usart2_buf[1], this_time_rx_len);
            }
#else
						fifo_s_puts(&referee_fifo, (char*)usart2_buf[0], this_time_rx_len);
#endif
        }
        else
        {
            __HAL_DMA_DISABLE(&hdma_usart2_rx);


            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart2_rx.Instance->NDTR;

            hdma_usart2_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
					
            __HAL_DMA_ENABLE(&hdma_usart2_rx);
					
#if		!RC_FIFO_ENABLE
            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(usart2_buf[1], &rc_ctrl);
            }
		
						if(this_time_rx_len == SELF_CONTROL_FRAME_LENGTH)
            {	
								fifo_s_puts(&referee_fifo, (char*)usart2_buf[1], this_time_rx_len);
            }
#else
						fifo_s_puts(&referee_fifo, (char*)usart2_buf[1], this_time_rx_len);
#endif
        }
    }
		HAL_UART_IRQHandler(&huart2);

}

