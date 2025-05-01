#include "crc_ccitt_modify.h"
#include "arm_control_task.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "string.h"
#include "bsp_go.h"
#include "usart.h"
#include "main.h"


static uint8_t TxBuffer_GO_0[17];
static uint8_t TxBuffer_GO_1[17];
uint8_t RxBuffer_GO_0[16];
uint8_t RxBuffer_GO_1[16];


//关节电机位置控制
void GO_motor_position_control(motor_GO_t *GO_motor_position,int i,fp32 pos)
{
		GO_motor_position->motor_send.header.HAED1 = 0xFE;
		GO_motor_position->motor_send.header.HAED2 = 0xEE;
		GO_motor_position->motor_send.header.RESERVE = 0x10+i;
		
		GO_motor_position->motor_send.trans_message.T = 0.0								*256;
		GO_motor_position->motor_send.trans_message.W = 0.0f							*6.33f*256.0f/(2.0f*PI);
		GO_motor_position->motor_send.trans_message.Pos = (int32_t)(pos		*6.33f*32768.0f/(2.0f*PI));
	
		if(i == 0)
		{
				GO_motor_position->motor_send.trans_message.K_P = 13.0						*1280;
				GO_motor_position->motor_send.trans_message.K_W = 0.15						*1280;			
		}
		else
		{
				GO_motor_position->motor_send.trans_message.K_P = 7.0						  *1280;
				GO_motor_position->motor_send.trans_message.K_W = 0.09						*1280;			
		}			
}


//关节电机速度控制
void GO_motor_velocity_control(motor_GO_t *GO_motor_velocity,int i,fp32 w)
{
		GO_motor_velocity->motor_send.header.HAED1 = 0xFE;
		GO_motor_velocity->motor_send.header.HAED2 = 0xEE;
		GO_motor_velocity->motor_send.header.RESERVE = 0x10+i;
		
		GO_motor_velocity->motor_send.trans_message.T = 0.0								*256;
		GO_motor_velocity->motor_send.trans_message.W =(int16_t)(w				*6.33f*256.0f/(2.0f*PI));
		GO_motor_velocity->motor_send.trans_message.Pos = 0.0f						*6.33f*32768.0f/(2.0f*PI);
		GO_motor_velocity->motor_send.trans_message.K_P = 0.0f						*1280;
	
		if(i == 0)
		{
				GO_motor_velocity->motor_send.trans_message.K_W = 0.125				  *1280;			
		}
		else
		{
				GO_motor_velocity->motor_send.trans_message.K_W = 0.075					*1280;			
		}			
}


//关节电机扭矩控制
void GO_motor_T_control(motor_GO_t *GO_motor_torque,int i,fp32 torque)
{
		GO_motor_torque->motor_send.header.HAED1 = 0xFE;
		GO_motor_torque->motor_send.header.HAED2 = 0xEE;
		GO_motor_torque->motor_send.header.RESERVE = 0x10+i;
			
		GO_motor_torque->motor_send.trans_message.T = torque						*256;
		GO_motor_torque->motor_send.trans_message.W = 0.0f							*6.33f*256.0f/(2.0f*PI);
		GO_motor_torque->motor_send.trans_message.Pos = 0.0f;
		GO_motor_torque->motor_send.trans_message.K_P = 0.0							*1280;
		GO_motor_torque->motor_send.trans_message.K_W = 0.0							*1280;				
}

//关节电机无力模式
void GO_motor_zero_force_control(motor_GO_t *GO_motor_zero_force,int i)
{
		GO_motor_zero_force->motor_send.header.HAED1 = 0xFE;
		GO_motor_zero_force->motor_send.header.HAED2 = 0xEE;
		GO_motor_zero_force->motor_send.header.RESERVE = 0x10+i;

		GO_motor_zero_force->motor_send.trans_message.T = 0.0									*256;
		GO_motor_zero_force->motor_send.trans_message.W = 0.0f								*6.33f*256.0f/(2.0f*PI);
		GO_motor_zero_force->motor_send.trans_message.Pos = (int32_t)(0.0f		*6.33f*32768.0f/(2.0f*PI));
		GO_motor_zero_force->motor_send.trans_message.K_P = 0.0								*1280;
		GO_motor_zero_force->motor_send.trans_message.K_W = 0.0								*1280;			
}

void modify_and_send_data_0(void)
{
		memcpy(TxBuffer_GO_0,&arm.motor_GO_data[0].motor_send.header, 3);
		memcpy(TxBuffer_GO_0+3, &arm.motor_GO_data[0].motor_send.trans_message,12);
		
		//计算CRC校验码2byte
		arm.motor_GO_data[0].motor_send.CRCdata.crc_u = crc_ccitt_modify(0x0000, TxBuffer_GO_0, 15);
		
		//结构体拷贝到buffer
		memcpy(TxBuffer_GO_0+15, &arm.motor_GO_data[0].motor_send.CRCdata,2);
		
		HAL_UART_Transmit_DMA(&huart1,TxBuffer_GO_0,17);
}

void modify_and_send_data_1(void)
{
		memcpy(TxBuffer_GO_1,&arm.motor_GO_data[1].motor_send.header, 3);
		memcpy(TxBuffer_GO_1+3, &arm.motor_GO_data[1].motor_send.trans_message,12);
		
		//计算CRC校验码2byte
		arm.motor_GO_data[1].motor_send.CRCdata.crc_u = crc_ccitt_modify(0x0000, TxBuffer_GO_1, 15);
		
		//结构体拷贝到buffer
		memcpy(TxBuffer_GO_1+15, &arm.motor_GO_data[1].motor_send.CRCdata,2);
		
		HAL_UART_Transmit_DMA(&huart6,TxBuffer_GO_1,17);	
}


void motor_solution_GO_0(void)
{
		uint16_t crc_temp_GO_0 = 0;
		uint8_t temp_GO_0[14];	
		rx_message_temp rx_temp;
	
		memcpy(temp_GO_0,RxBuffer_GO_0,14);
			
		//判断CRC校验码2byte
		crc_temp_GO_0 = crc_ccitt_modify(0x0000, temp_GO_0, 14);
		if(crc_temp_GO_0 == (RxBuffer_GO_0[14] | RxBuffer_GO_0[15]<<8))
		{
				memcpy(&arm.motor_GO_data[0].motor_recv.header , RxBuffer_GO_0,3);
				memcpy(&rx_temp , RxBuffer_GO_0+3, 11);			
				//解包
				arm.motor_GO_data[0].motor_recv.trans_message.T = (float)rx_temp.T/256.0f;
				arm.motor_GO_data[0].motor_recv.trans_message.W = (float)rx_temp.W/6.33f/256.0f*2*PI;
				arm.motor_GO_data[0].motor_recv.trans_message.Pos = (float)rx_temp.Pos/6.33f/32768.0f*2*PI;
				//-128.0~127.0,90度保护
				arm.motor_GO_data[0].motor_recv.trans_message.Temp	= (float)rx_temp.Temp;
				//0.正常1.过热2.过流3.过压4.编码器故障
				arm.motor_GO_data[0].motor_recv.trans_message.MERROR = (float)(rx_temp.Reserve >> 13);
				//0-4095
				arm.motor_GO_data[0].motor_recv.trans_message.Force = (float)((rx_temp.Reserve >> 1) & 0xFFF);
		}
}

 void motor_solution_GO_1(void)
{
		uint16_t crc_temp_GO_1 = 0;
		uint8_t temp_GO_1[14];
		rx_message_temp rx_temp;
	
		memcpy(temp_GO_1,RxBuffer_GO_1,14);
		
		//判断CRC校验码2byte
		crc_temp_GO_1 = crc_ccitt_modify(0x0000, temp_GO_1, 14);
		if(crc_temp_GO_1 == (RxBuffer_GO_1[14] | RxBuffer_GO_1[15]<<8))
		{
				memcpy(&arm.motor_GO_data[1].motor_recv.header, RxBuffer_GO_1,3);				
				memcpy(&rx_temp , RxBuffer_GO_1+3, 11);
				
				//解包
				arm.motor_GO_data[1].motor_recv.trans_message.T = (float)rx_temp.T/256.0f;
				arm.motor_GO_data[1].motor_recv.trans_message.W = (float)rx_temp.W/6.33f/256.0f*2*PI;
				arm.motor_GO_data[1].motor_recv.trans_message.Pos = (float)rx_temp.Pos/6.33f/32768.0f*2*PI;
				//-128.0~127.0,90度保护
				arm.motor_GO_data[1].motor_recv.trans_message.Temp	= (float)rx_temp.Temp;
				//0.正常1.过热2.过流3.过压4.编码器故障
				arm.motor_GO_data[1].motor_recv.trans_message.MERROR = (float)(rx_temp.Reserve >> 13);
				//0-4095
				arm.motor_GO_data[1].motor_recv.trans_message.Force = (float)((rx_temp.Reserve >> 1) & 0xFFF);
		}
}


void USART1_IRQHandler(void)  
{

		/* USER CODE END USART1_IRQn 0 */
		HAL_UART_IRQHandler(&huart1);
		/* USER CODE BEGIN USART1_IRQn 1 */
		if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
		{
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);
				HAL_UART_Receive_DMA(&huart1,(uint8_t *)RxBuffer_GO_0,16);	
				motor_solution_GO_0();					 
		}
		/* USER CODE END USART1_IRQn 1 */
}

void USART6_IRQHandler(void)
{
	
		if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))
		{
				__HAL_UART_CLEAR_IDLEFLAG(&huart6);			
				HAL_UART_Receive_DMA(&huart6,(uint8_t *)RxBuffer_GO_1,16);
				motor_solution_GO_1();
		}
		
		/* USER CODE END USART6_IRQn 0 */
		HAL_UART_IRQHandler(&huart6);
		/* USER CODE BEGIN USART6_IRQn 1 */

		/* USER CODE END USART6_IRQn 1 */
}

