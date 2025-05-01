#include "pc_task.h"
#include "cmsis_os.h"
#include "shoot.h"
#include "gimbal_task.h"
#include "can_communicate.h"
#include "protocol.h"
#include "bsp_usart.h"

send_msg_t pc_send_msg;
receive_msg_t pc_receive_msg;
//接收原始数据，为12个字节，给了24个字节长度，防止DMA传输越界
//static uint8_t PC_rx_buf[2][PC_RX_BUF_NUM];
uint8_t PC_SEND_BUF[LEN_TX_PACKET + 1];
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
void pc_task(void const *argu)
{
	 // usart6_init(PC_rx_buf[0],PC_rx_buf[1],PC_RX_BUF_NUM);
	 usart6_idle_init();
   usart6_tx_dma_init();
	  vTaskDelay(100);
	//获取当前系统时间，方便延时用
    uint32_t mode_wake_time = osKernelSysTick();
    while(1)
    {
        //对各数据赋值
        pc_send_msg.frame_header.sof = FRAME_HEADER; //帧头
        pc_send_msg.tx_data.curr_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;     //当前云台yaw角度
        pc_send_msg.tx_data.curr_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle; //当前云台pitch角度
        pc_send_msg.tx_data.state = '0' + gimbal_control.gimbal_remote_data->key_b.key_switch; //当前状态，0自瞄-1打符
        if(gimbal_control.gimbal_mode == GIMBAL_MODE_PC)
		    pc_send_msg.tx_data.pc_state = 1;
		    else
		    pc_send_msg.tx_data.pc_state = 0;			
			  if(color_flag == 0) pc_send_msg.tx_data.enemy_color = 0;
        if(color_flag == 1) pc_send_msg.tx_data.enemy_color = 1;
        //在帧头和帧尾加入CRC校验，其中CRC8校验函数可以不调用
        append_crc8_check_sum(&pc_send_msg.frame_header.sof, LEN_FRAME_HEADER);
        append_crc16_check_sum(&pc_send_msg.frame_header.sof, LEN_TX_PACKET);
        //发送数据
        memcpy(PC_SEND_BUF, &pc_send_msg, sizeof(pc_send_msg));
        PC_SEND_BUF[LEN_TX_PACKET]= '\n';
        usart6_tx_dma_enable(PC_SEND_BUF,LEN_TX_PACKET+1);
        osDelayUntil(&mode_wake_time, 7);
    }
}
float before_yaw = 0, before_pitch = 0;int for_pitch = 0;
static void data_solve(receive_msg_t *pc_receive_msg, uint8_t *rx_data)
{
	  static uint8_t res = 0;
	if(rx_data[SOF_ADDR] == FRAME_HEADER) 
	{	
		//帧尾CRC16校验
		res = verify_crc16_check_sum(rx_data, LEN_RX_PACKET);
		if(res == 1) 
		{
			//数据正确则拷贝接收数据
			memcpy(pc_receive_msg, rx_data, LEN_RX_PACKET);	
		}
	}
	//若数据无效，保持原数据不变
	if(res == 0) 
	{
		pc_receive_msg->rx_data.shoot_yaw = before_yaw;
		pc_receive_msg->rx_data.shoot_pitch = before_pitch;
	}
	else
	{
		//记录这次接收到的角度值
		before_yaw = pc_receive_msg->rx_data.shoot_yaw;
		before_pitch = pc_receive_msg->rx_data.shoot_pitch;
		for_pitch = 1000*pc_receive_msg->rx_data.shoot_pitch;
	}
}

//触发空闲中断时进行数据的校验与拷贝
//void USART6_RX_IRQHandler(void)
//{
//    if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE)) //&& __HAL_UART_GET_IT_SOURCE(&huart6, UART_IT_IDLE))
//    {
//        __HAL_UART_CLEAR_IDLEFLAG(&huart6);
//        uint32_t DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart6.hdmarx);
//        USART6_RX_STA = USART6_MAX_RECV_LEN - huart6.hdmarx->Instance->NDTR;
//        __HAL_DMA_DISABLE(huart6.hdmarx);
//        data_solve(&pc_receive_msg,USART6_RX_BUF);
//        __HAL_DMA_CLEAR_FLAG(huart6.hdmarx, DMA_FLAGS);
//        __HAL_DMA_SET_COUNTER(huart6.hdmarx,USART6_MAX_RECV_LEN);
//        __HAL_DMA_ENABLE(huart6.hdmarx);
//    }
//}
//串口中断
//void USART6_RX_IRQHandler(void)
//{
//    if(huart6.Instance->SR & UART_FLAG_RXNE)//接收到数据
//    {
//        __HAL_UART_CLEAR_PEFLAG(&huart6);
//    }
//    else if(USART6->SR & UART_FLAG_IDLE)
//    {
//        static uint16_t this_time_rx_len = 0;

//        __HAL_UART_CLEAR_PEFLAG(&huart6);

//        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
//        {
//            /* Current memory buffer used is Memory 0 */

//            //disable DMA
//            //失效DMA
//            __HAL_DMA_DISABLE(&hdma_usart6_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //获取接收数据长度,长度 = 设定长度 - 剩余长度
//            this_time_rx_len = PC_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

//            //reset set_data_lenght
//            //重新设定数据长度
//            hdma_usart6_rx.Instance->NDTR = PC_RX_BUF_NUM;

//            //set memory buffer 1
//            //设定缓冲区1
//            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;

//            //enable DMA
//            //使能DMA
//            __HAL_DMA_ENABLE(&hdma_usart6_rx);

//            if(this_time_rx_len == PC_RX_BUF_LENGTH)
//            {
//                data_solve(&pc_receive_msg,PC_rx_buf[0]);
//            }
//        }
//        else
//        {
//            /* Current memory buffer used is Memory 1 */
//            //disable DMA
//            //失效DMA
//            __HAL_DMA_DISABLE(&hdma_usart6_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //获取接收数据长度,长度 = 设定长度 - 剩余长度
//            this_time_rx_len = PC_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

//            //reset set_data_lenght
//            //重新设定数据长度
//            hdma_usart6_rx.Instance->NDTR =PC_RX_BUF_NUM;

//            //set memory buffer 0
//            //设定缓冲区0
//            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

//            //enable DMA
//            //使能DMA
//            __HAL_DMA_ENABLE(&hdma_usart6_rx);

//            if(this_time_rx_len == PC_RX_BUF_LENGTH)
//            {
//                //处理遥控器数据
//                data_solve(&pc_receive_msg,PC_rx_buf[1]);

//            }
//        }
//    }
//}
const rx_data_t *get_pc_rx_point(void)
{
    return &pc_receive_msg.rx_data;
}
