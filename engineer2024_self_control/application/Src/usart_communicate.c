#include "usart_communicate.h"
#include "6dof_kinematic.h"
#include "usart.h"
#include "string.h"
#include "motor_task.h"
#include "imu_task.h"
#include "cmsis_os.h"
uint8_t seq = 0;
send_data send_data_self_control;
uint8_t tx_buffer_self_control[39];
const float32_t *IMU_Q_Point;
//tx_self_control_message tx_self_control;
float32_t IMU_Q_0[4];//C板默认坐标系
float32_t IMU_Q_1[4];//自定义控制器默认坐标系
mat IMU_R_0[9],IMU_R_1[9];
mat R_Trans[9];//c板默认坐标系到自定义控制器默认坐标系
float *IMU_R_0_data,*IMU_R_1_data,*R_Trans_data;

float euler[3];
float qua[4];

static void tx_init(void);

void usart_communicate_task(void const * argument)
{
	osDelay(2000);
	tx_init();
	IMU_R_0_data = (float *)user_malloc(sizeof(float) * 3 * 3);
	memset(IMU_R_0_data, 0, sizeof(float) * 3 * 3);
	Matrix_Init(IMU_R_0,3, 3,(float *)IMU_R_0_data);
	IMU_R_1_data = (float *)user_malloc(sizeof(float) * 3 * 3);
	memset(IMU_R_1_data, 0, sizeof(float) * 3 * 3);	
	Matrix_Init(IMU_R_1,3, 3,(float *)IMU_R_1_data);
	R_Trans_data = (float *)user_malloc(sizeof(float) * 3 * 3);
	memset(R_Trans_data, 0, sizeof(float) * 3 * 3);
	Matrix_Init(R_Trans,3, 3,(float *)R_Trans_data);
	fp32 trans_temp[9] = {1,0,0,
												0,0,-1,
												0,1,0};
	memcpy(R_Trans_data,trans_temp,36);
	while(1)
	{
		osDelay(40);
		send_data_self_control.header.seq = seq++;
		IMU_Q_Point = get_imu_quat_point();
		for(int i=0; i<3; i++)
		{
			send_data_self_control.data[i] = self_control_motor_position[i];
		}
		
		for(int i=0; i<4; i++)
		{
			IMU_Q_0[i]= (*(IMU_Q_Point+i));
		}
		QuaToRotMat(IMU_Q_0,IMU_R_0_data);
		Matrix_Multiply(R_Trans,IMU_R_0,IMU_R_1);
		RotMatToQua(IMU_R_1_data,IMU_Q_1);
		for(int i=0; i<4; i++)
		{
			send_data_self_control.data[i+3] = IMU_Q_1[i];
			QuaToEulerAngle(IMU_Q_1,euler);
		}
		append_crc8_check_sum(&send_data_self_control.header.sof, 5);
		append_crc16_check_sum(&send_data_self_control.header.sof, 39);
		memcpy(tx_buffer_self_control,&send_data_self_control, 39);
		HAL_UART_Transmit_DMA(&huart1,tx_buffer_self_control,39);
		
	}
}

void tx_init()
{
	  send_data_self_control.header.sof = 0xA5;
    send_data_self_control.header.dataLenth = 30; // 4 * 7 + 1 * 2 = 30
    send_data_self_control.header.seq = 0;
		send_data_self_control.cmd_id = 0x0302;
}


void USART6_RX_IRQHandler(void)
{
}


