#include "pc_speed.h"
#include "main.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "string.h"
#include "CRC.h"
#include "imu_task.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"
#include "stm32f4xx_it.h"
#include "stdarg.h"
#include "gimbal_task.h"
#include "can_solve.h"
vel_receive_msg_t vel_receive_msg;
imu_data_t send_imu_data;

uint8_t PC_SEND_IMU_BUF[sizeof(imu_data_t) + 20]; 
//static uint8_t seqcount = 0;
static uint8_t res = 0;
vel_rx_data_t vel_ctrl;
float32_t time_ms;
float32_t global_time_ms;
uint64_t run_dog;
static const float32_t imu_filter[3] = {1.822751349748f, -0.8371810677374f,  0.01442971798952f};
static float32_t yaw_gyro_fliter[3] = {0.0f, 0.0f, 0.0f};
uint8_t odom_reset;
//static void data_solve_vel(vel_receive_msg_t *vel_receive_msg, uint8_t *rx_data)
//{
//			global_time_ms = Get_SystemTimer() * 10;
//			
//            memcpy(vel_receive_msg, rx_data, sizeof(vel_receive_msg_t));
//			if(rx_data[0] == 'v' && rx_data[1] == 'e' && rx_data[2] == 'l' && rx_data[3] == '|'  && rx_data[5] == '|'  && rx_data[10] == '|'  && rx_data[15] == '|'  && rx_data[18] == '.' && rx_data[21] == '|' )
//			{	
//				vel_ctrl.is_continue =  (rx_data[4]-'0');
//				if(rx_data[6] == '-')
//					vel_ctrl.vx = -((rx_data[7]-'0')*100 + (rx_data[8]-'0')*10 + (rx_data[9]-'0'));
//				else
//					vel_ctrl.vx = (rx_data[7]-'0')*100 + (rx_data[8]-'0')*10 + (rx_data[9]-'0');
//				if(rx_data[11] == '-')
//					vel_ctrl.vy = -((rx_data[12]-'0')*100 + (rx_data[13]-'0')*10 + (rx_data[14]-'0'));
//				else
//					vel_ctrl.vy = (rx_data[12]-'0')*100 + (rx_data[13]-'0')*10 + (rx_data[14]-'0');
//				if(rx_data[16] == '-')
//					vel_ctrl.vw = ((rx_data[17]-'0') + 0.1f * (rx_data[19]-'0') + 0.01f * ((float32_t)(rx_data[20]-'0')));
//				else
//					vel_ctrl.vw = -(rx_data[17]-'0') + 0.1f * (rx_data[19]-'0') + 0.01f * ((float32_t)(rx_data[20]-'0'));
//		    }
//		   run_dog = 0;
//				
//}

int16_t vx=0,vy=0;
float32_t wz=0;
static void data_solve_vel(vel_receive_msg_t *vel_receive_msg, uint8_t *rx_data)
{
			global_time_ms = Get_SystemTimer() * 10;
			
            memcpy(vel_receive_msg, rx_data, sizeof(vel_receive_msg_t));
			if(rx_data[0] == 'v' && rx_data[1] == 'e' && rx_data[2] == 'l' && rx_data[3] == '|'  && rx_data[5] == '|'  && rx_data[10] == '|'  && rx_data[15] == '|'  && rx_data[18] == '.' )
			{	
				vel_ctrl.is_continue =  (rx_data[4]-'0');
				if(rx_data[6] == '-')
					vel_ctrl.vx = -((rx_data[7]-'0')*100 + (rx_data[8]-'0')*10 + (rx_data[9]-'0'));
				else
					vel_ctrl.vx = (rx_data[7]-'0')*100 + (rx_data[8]-'0')*10 + (rx_data[9]-'0');
				if(rx_data[11] == '-')
					vel_ctrl.vy = -((rx_data[12]-'0')*100 + (rx_data[13]-'0')*10 + (rx_data[14]-'0'));
				else
					vel_ctrl.vy = (rx_data[12]-'0')*100 + (rx_data[13]-'0')*10 + (rx_data[14]-'0');
				if(rx_data[16] == '-')
					vel_ctrl.vw = -((rx_data[17]-'0') + 0.1f * (rx_data[19]-'0') + 0.01f * ((float32_t)(rx_data[20]-'0')));
				else
					vel_ctrl.vw = (rx_data[17]-'0') + 0.1f * (rx_data[19]-'0') + 0.01f * ((float32_t)(rx_data[20]-'0'));
		  }
//			 vel_ctrl.vx = 0.5 * vx + 0.5 * vel_ctrl.vx;
//		   vel_ctrl.vy = 0.5 * vy + 0.5 * vel_ctrl.vy;
			 vel_ctrl.vw = 0.4777*vel_ctrl.vw;//0.5 * wz + 0.5 * vel_ctrl.vw;
//			vx = vel_ctrl.vx;
//			vy = vel_ctrl.vy;
//			wz = vel_ctrl.vw;
}

//发送imu数据
void pc_send_imu_task(void const *argu)
{
	uint32_t mode_wake_time = osKernelSysTick();

	while(1)
	{
		yaw_gyro_fliter[0] = yaw_gyro_fliter[1];
		yaw_gyro_fliter[1] = yaw_gyro_fliter[2];
		yaw_gyro_fliter[2] = yaw_gyro_fliter[1] * imu_filter[0] + yaw_gyro_fliter[0] * imu_filter[1] +  gimbal_control.gimbal_yaw_motor.motor_gyro * imu_filter[2];
		//串口imu
		send_imu_data.sof = 'i';
		send_imu_data.q0 = *(get_imu_quat_point());
		send_imu_data.q1 = *(get_imu_quat_point() + 1);
		send_imu_data.q2 = *(get_imu_quat_point() + 2);
		send_imu_data.q3 = *(get_imu_quat_point() + 3);
		send_imu_data.gyro_x  = *(get_gyro_data_point());
		send_imu_data.gyro_y  = *(get_gyro_data_point() +1);
		send_imu_data.gyro_z  = yaw_gyro_fliter[2];
		send_imu_data.accel_x = *(get_accel_data_point());
		send_imu_data.accel_y = *(get_accel_data_point() +1);
		send_imu_data.accel_z = *(get_accel_data_point() +2);
		send_imu_data.end = 'e';
		//串口通信
		//crc校验类型 CCITT_FALSE
		memcpy(PC_SEND_IMU_BUF, &send_imu_data, sizeof(imu_data_t));
//		HAL_UART_Transmit(&huart6, PC_SEND_IMU_BUF, sizeof(imu_data_t), 1000);
		osDelayUntil(&mode_wake_time, 50);
	}
}

void USART6_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart6);
    if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(&huart6, UART_IT_IDLE))
    {
         __HAL_UART_CLEAR_IDLEFLAG(&huart6);
         uint32_t DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart6.hdmarx); 
        USART6_RX_STA = USART6_MAX_RECV_LEN - huart6.hdmarx->Instance->NDTR; 
        __HAL_DMA_DISABLE(huart6.hdmarx);
        data_solve_vel(&vel_receive_msg, USART6_RX_BUF);
        __HAL_DMA_CLEAR_FLAG(huart6.hdmarx, DMA_FLAGS);	
		__HAL_DMA_SET_COUNTER(huart6.hdmarx,USART6_MAX_RECV_LEN);
		__HAL_DMA_ENABLE(huart6.hdmarx);
    }
}

vel_rx_data_t *get_pc_vel(void)
{
    return &vel_ctrl;
}
