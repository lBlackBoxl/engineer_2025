#include "usart_communicate.h"
#include "usart.h"
#include "string.h"
#include "motor_task.h"
#include "6dof_kinematic.h"
#include "ins_task.h"

extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

send_msg_t imu_send_msg;
uint8_t tx_buffer[8];
const fp32 *gimbal_INT_angle_point;
const fp32 *gimbal_INT_gyro_point;
uint8_t Robot_state[20];

uint8_t USART6_RX_Buffer[USART6_MAX_RECV_LEN];

uint8_t chassis_mode;
uint8_t arm_mode;
uint8_t move_mode;
uint8_t suker_key_flag;
uint8_t Ore_1_flag;
uint8_t Ore_2_flag;
uint8_t clamp_flag;
uint8_t AJX_flag;
uint8_t arm_restart_flag;

void data_solve(uint8_t *usart_buffer,uint8_t *rx_buffer);
static int UART_Receive_DMA_No_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);
void USART6_IDLE_Enable(void);

void usart_communicate_task(void const * argument)
{

    //陀螺仪数据指针获取
  gimbal_INT_angle_point = get_angle_data_point();
	USART6_IDLE_Enable();
	
	while(1)
	{
		imu_send_msg.frame_header.sof 		= 0xA5;
		imu_send_msg.tx_data.yaw_angle    = *(gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
		
		append_crc8_check_sum(&imu_send_msg.frame_header.sof, 2);		
		append_crc16_check_sum(&imu_send_msg.frame_header.sof, 8);
		memcpy(tx_buffer,&imu_send_msg, 8);

		HAL_UART_Transmit_DMA(&huart6,tx_buffer,8);
		osDelay(2);		
	}
}

void USART6_RX_IRQHandler(void)
{
	uint8_t USART6_RX_STA = 0;
	if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(&huart6, UART_IT_IDLE))
	{
		__HAL_DMA_DISABLE(huart6.hdmarx);
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);
		uint32_t DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart6.hdmarx); 
		USART6_RX_STA = USART6_MAX_RECV_LEN - huart6.hdmarx->Instance->NDTR; 
		data_solve(USART6_RX_Buffer, Robot_state);
		__HAL_DMA_CLEAR_FLAG(huart6.hdmarx, DMA_FLAGS);	
		__HAL_DMA_SET_COUNTER(huart6.hdmarx,USART6_MAX_RECV_LEN);
		__HAL_DMA_ENABLE(huart6.hdmarx);
	}
}

void USART6_IDLE_Enable(void)
{
		huart6.Instance = USART6;
		huart6.Init.BaudRate = 115200;
		huart6.Init.WordLength = UART_WORDLENGTH_8B;
		huart6.Init.StopBits = UART_STOPBITS_1;
		huart6.Init.Parity = UART_PARITY_NONE;
		huart6.Init.Mode = UART_MODE_TX_RX;
		huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		huart6.Init.OverSampling = UART_OVERSAMPLING_16;
		if (HAL_UART_Init(&huart6) != HAL_OK)
		{
				Error_Handler();
		}
		
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);
		__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);									//使能空闲中断
		UART_Receive_DMA_No_IT(&huart6, USART6_RX_Buffer , USART6_MAX_RECV_LEN);
}

void data_solve(uint8_t *usart_buffer,uint8_t *rx_buffer)
{
	for(int i = 0; i < USART6_MAX_RECV_LEN; i++)
	{
		if(usart_buffer[i] == 'i') 
		{	
			//帧尾CRC16校验
			if(usart_buffer[i + 19] == 'e')
			{
				memcpy(rx_buffer, &usart_buffer[i], 20);	
				break;
			}
		}
	}
	chassis_mode = rx_buffer[1];
	arm_mode = rx_buffer[2];
	move_mode = rx_buffer[3];
	suker_key_flag = rx_buffer[4];
	Ore_1_flag = rx_buffer[5];
	Ore_2_flag = rx_buffer[6];
	clamp_flag = rx_buffer[7];
	AJX_flag = rx_buffer[8];
	arm_restart_flag = rx_buffer[9];
}

static int UART_Receive_DMA_No_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
  uint32_t tmp1 = 0;
  tmp1 = huart->RxState;
  if (tmp1 == HAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0))
    {
        return HAL_ERROR;
    }
    __HAL_LOCK(huart);
    huart->pRxBuffPtr = pData;
    huart->RxXferSize = Size;
    huart->ErrorCode  = HAL_UART_ERROR_NONE;
    HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR,(uint32_t)pData, Size);
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    __HAL_UNLOCK(huart);
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}
