/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-01-2019     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
	
#include "remote_control.h"
#include "main.h"
#include "bsp_rc.h"
#include "keyboard.h"
//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

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

//遥控器控制变量
RC_ctrl_t rc_ctrl;
key_t last_s[5];
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
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

//串口中断
void USART1_IRQHandler(void)
{
    if(huart1.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if(USART1->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;


            hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;

            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        }
        else
        {
            __HAL_DMA_DISABLE(&hdma_usart1_rx);


            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
					
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }
        }
    }
		HAL_UART_IRQHandler(&huart1);

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
		
		uint16_t key_information;
		key_information = (sbus_buf[17] | (sbus_buf[18] << 8));                 //键盘数据
		
    rc_ctrl->rc.ch[0] = (sbus_buf[2] | (sbus_buf[3] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[3] >> 3) | (sbus_buf[4] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 6) | (sbus_buf[5] << 2) |          //!< Channel 2
                         (sbus_buf[6] << 10)) &0x07ff;
    rc_ctrl->rc.ch[2] = ((sbus_buf[6] >> 1) | (sbus_buf[7] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[1] = ((sbus_buf[7] >> 4) & 0x0003);                   //挡位切换开关
    //rc_ctrl->rc.s[0] = ((sbus_buf[7] >> 6) & 0x0001);                       //暂停按键
		rc_ctrl->rc.s[2] = ((sbus_buf[7] >> 7) & 0x0001);												//自定义按键（left）
		rc_ctrl->rc.s[3] = (sbus_buf[8] & 0x0001);														  //自定义按键（right）
		if(rc_ctrl->rc.s[2] == 0 &&  rc_ctrl->rc.s[3] == 0)
		{
			rc_ctrl->rc.s[0] = 1;
		}
		else if(rc_ctrl->rc.s[2] == 1 &&  rc_ctrl->rc.s[3] == 0)
		{
			rc_ctrl->rc.s[0] = 2;
		}
		else if(rc_ctrl->rc.s[2] == 0 &&  rc_ctrl->rc.s[3] == 1)
		{
			rc_ctrl->rc.s[0] = 0;
		}
		else if(rc_ctrl->rc.s[2] == 1 &&  rc_ctrl->rc.s[3] == 1)
		{
			rc_ctrl->rc.s[0] = 1;
		}
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
		
		for(int i = 0; i < 5; i++)
		{
				last_s[i].itself.last_mode = last_s[i].itself.mode;
				last_s[i].content = rc_ctrl->rc.s[i];
				if(last_s[i].itself.flag == 0)
				{
					if(last_s[i].content != 0)
					{
							last_s[i].itself.time++;
					}
					if(last_s[i].itself.time >= 10) 
					{	
						last_s[i].itself.flag = 1;
						last_s[i].itself.time = 0;
					}
				}
				else                                        
				{
					if(last_s[i].content  == 0)   
					{
							last_s[i].itself.time++;
					}
					if(last_s[i].itself.time >= 10) 
					{	
							last_s[i].itself.flag = 0;
							last_s[i].itself.time = 0;
							last_s[i].itself.mode = last_s[i].itself.mode +1;
							if(last_s[i].itself.mode == 2)
							last_s[i].itself.mode=0;
					}
				}
		}
}
