#include "pc_auto_aim.h"
#include "gimbal_task.h"
#include "bsp_usart.h"
#include "CRC.h"
#include "string.h"
#include "cmsis_os.h"
#include "main.h"
#include "shoot_task.h"
#include "pc_speed.h"

auto_aim_send_msg_t auto_aim_send_msg;
auto_aim_receive_msg_t auto_aim_receive_msg;
uint8_t PC_SEND_BUF[sizeof(auto_aim_send_msg_t)+1];
static uint8_t seqcount = 0;
static uint16_t dog = 0;
void pc_auto_aim_task(void const *argu)
{
    //获取当前系统时间，方便延时用
	uint8_t i;
	uint32_t mode_wake_time = osKernelSysTick();

	
    while(1)
    {
		for(i=0;i<8;i++)
			auto_aim_send_msg.tx_data.hit[i] = 1;
		auto_aim_send_msg.tx_data.hit[1] = 1;
		auto_aim_send_msg.tx_data.hit[5] = 1;
		auto_aim_send_msg.tx_data.hit[6] = 0;
		auto_aim_send_msg.tx_data.hit[7] = 0;
	
		auto_aim_send_msg.tx_data.is_balance[0] =0;
		auto_aim_send_msg.tx_data.is_balance[1] =0;
		auto_aim_send_msg.tx_data.is_balance[2] = 0;
        auto_aim_send_msg.frame_header.sof = 0xA5;
		if(shoot_control.bullet_speed < 15.0f) shoot_control.bullet_speed = 27.5f;
        auto_aim_send_msg.tx_data.shoot_speed = shoot_control.bullet_speed; //bullet_speed;
        auto_aim_send_msg.tx_data.curr_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;
		auto_aim_send_msg.tx_data.curr_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
		if(run_dog > 1000)
			auto_aim_send_msg.tx_data.state = '0';
		else
			auto_aim_send_msg.tx_data.state = '1';
		//发送敌人颜色
		if(color_flag == 1)
			auto_aim_send_msg.tx_data.enemy_color = '1';
		else
			auto_aim_send_msg.tx_data.enemy_color = '0';
        append_crc8_check_sum(&auto_aim_send_msg.frame_header.sof, LEN_FRAME_HEADER);
        append_crc16_check_sum(&auto_aim_send_msg.frame_header.sof, sizeof(auto_aim_send_msg_t));
        //发送数据
        memcpy(PC_SEND_BUF, &auto_aim_send_msg, sizeof(auto_aim_send_msg));
        PC_SEND_BUF[sizeof(auto_aim_send_msg_t)] = '\n';
        HAL_UART_Transmit(&huart1, PC_SEND_BUF, sizeof(auto_aim_send_msg_t) + 1, 2000);
        osDelayUntil(&mode_wake_time, 7);
		dog++;
		if(dog > 800) auto_aim_receive_msg.rx_data.fire = 0;
    }

}

static uint8_t res = 0;
float32_t before_yaw = 0, before_pitch = 0;
static void data_solve(auto_aim_receive_msg_t *auto_aim_receive_msg, uint8_t *rx_data)
{
    if(rx_data[SOF_ADDR] == FRAME_HEADER)
    {
        //帧尾crc16校验
        res = verify_crc16_check_sum(rx_data, sizeof(auto_aim_receive_msg_t));
        if(res == 1)
        {
            memcpy(auto_aim_receive_msg, rx_data, sizeof(auto_aim_receive_msg_t));
            //记录这次接收到的角度值
			before_yaw = auto_aim_receive_msg->rx_data.shoot_yaw;
			before_pitch = auto_aim_receive_msg->rx_data.shoot_pitch;
        }
        if(res == 0)
        {
            auto_aim_receive_msg->rx_data.shoot_yaw = before_yaw;
			auto_aim_receive_msg->rx_data.shoot_pitch = before_pitch;
        }
    }

	dog = 0;
}
//触发空闲中断时进行数据的校验和拷贝
void USART1_IRQHandler(void)
{
    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
		uint32_t DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart1.hdmarx); 
		USART1_RX_STA = USART1_MAX_RECV_LEN - huart1.hdmarx->Instance->NDTR; 
		__HAL_DMA_DISABLE(huart1.hdmarx);
		data_solve(&auto_aim_receive_msg,USART1_RX_BUF);
		__HAL_DMA_CLEAR_FLAG(huart1.hdmarx, DMA_FLAGS);	
		__HAL_DMA_SET_COUNTER(huart1.hdmarx,USART1_MAX_RECV_LEN);
		__HAL_DMA_ENABLE(huart1.hdmarx);
    }
}
