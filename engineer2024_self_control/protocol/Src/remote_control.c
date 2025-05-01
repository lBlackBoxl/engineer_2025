#include "remote_control.h"
#include "bsp_rc.h"
#include "string.h"
#include "main.h"
#include "detect_task.h"
#include "MATH_LIB.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
rc_ctrl_t rc_ctrl;
//遥控器缓存区
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/// @brief 遥控器原始数据解包
/// @param sbus_buf 遥控器原始数据
/// @param rc_ctrl 遥控器结构体
static void sbus_to_rc(volatile const uint8_t *sbus_buf, rc_ctrl_t *rc_ctrl);

//遥控器初始化
void remote_control_init(void)
{
    rc_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

const rc_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

//遥控器串口中断
void USART3_IRQHandler(void)
{
    //触发接收中断
    if(huart3.Instance->SR & UART_FLAG_RXNE)
    {
        //清除标志位
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    //触发空闲中断
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;
        //清除标志位
        __HAL_UART_CLEAR_PEFLAG(&huart3);
        //接收第一段数据
        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
             //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);
            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //将sbuff保存到rc_ctrl
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
                //记录数据接收时间
                detect_hook(DBUS_TOE);
            }
        }
        //接收第二段数据
        else
        {
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);
             //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);
            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
                 //记录数据接收时间
                detect_hook(DBUS_TOE);
            }
        }  
    }
}

static void sbus_to_rc(volatile const uint8_t *sbus_buf, rc_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }
    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //0通道值
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //1通道值
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //2通道值
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //3通道值
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                  //左拨杆值
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x0003);                       //右拨杆值
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //鼠标左右值
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //鼠标上下值
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //鼠标滚轮值
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //鼠标左键值
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //鼠标右键值
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //键盘
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                
    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}
