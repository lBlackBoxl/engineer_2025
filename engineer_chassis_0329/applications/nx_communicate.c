#include "FreeRTOS.h"
#include "main.h"
#include "usart.h"
#include "nx_communicate.h"
#include "cmsis_os.h"
#include "string.h"
#include "arm_control_task.h"
#include "boards_communicate.h"
#include "referee.h"

extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;

uint8_t  USART7_RX_BUF[USART7_MAX_RECV_LEN]; 
uint16_t USART7_RX_STA = 0;

nx_send_msg_t pc_send_msg;
//pc_info_test_key test_key;

nx_receive_msg_t pc_receive_msg;
uint8_t PC_SEND_BUF[LEN_TX_PACKET + 1];

float nx_allowance[6] = {0.5589f,-1.5935f,1.5933f,0.0f,-1.6356f,0.0575f};

extern void nx_communicate_init(void);
static int UART_Receive_DMA_No_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);
	
void nx_communicate_task(void const *pvParameters)
{
		vTaskDelay(500);
		
		pc_receive_msg.rx_data.motor1_position = -nx_allowance[0];
		pc_receive_msg.rx_data.motor2_position =  nx_allowance[1];
		pc_receive_msg.rx_data.motor3_position =  nx_allowance[2];
		pc_receive_msg.rx_data.motor4_position =  nx_allowance[3];
		pc_receive_msg.rx_data.motor5_position =  (nx_allowance[4] + 1.466f);
		pc_receive_msg.rx_data.motor6_position =  nx_allowance[5];
	
		nx_communicate_init();
		
		uint32_t mode_wake_time = osKernelSysTick();
		while(1)
		{   
				pc_send_msg.tx_data.motor1_position =  (arm_control.motor_YAW_data.DM_motor_measure->motor_position - nx_allowance[0]);
				pc_send_msg.tx_data.motor2_position =  (arm_message.target_position[1] + nx_allowance[1]);
				pc_send_msg.tx_data.motor3_position =  (-arm_message.target_position[2] + nx_allowance[2]);
				pc_send_msg.tx_data.motor4_position =  (arm_message.target_position[3] + nx_allowance[3]);
				pc_send_msg.tx_data.motor5_position = -(arm_message.target_position[4] + nx_allowance[4]);
				pc_send_msg.tx_data.motor6_position =  (arm_message.target_position[5] + nx_allowance[5]);
				pc_send_msg.robot_id = robot_state.robot_id;
				pc_send_msg.arm_move_flag = arm_control.arm_move_flag;
				
				pc_send_msg.frame_header.sof = 0x69;
				
				pc_send_msg.frame_tailer.crc16 = 0x65;
				memcpy(PC_SEND_BUF, &pc_send_msg, sizeof(pc_send_msg));
				PC_SEND_BUF[LEN_TX_PACKET]= '\n';
				HAL_UART_Transmit(&huart7, PC_SEND_BUF, LEN_TX_PACKET + 1, 2000);
				osDelayUntil(&mode_wake_time, 7);
		}
}

void nx_communicate_init(void)
{
		huart7.Instance = UART7;
		huart7.Init.BaudRate = 115200;
		huart7.Init.WordLength = UART_WORDLENGTH_8B;
		huart7.Init.StopBits = UART_STOPBITS_1;
		huart7.Init.Parity = UART_PARITY_NONE;
		huart7.Init.Mode = UART_MODE_TX_RX;
		huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		huart7.Init.OverSampling = UART_OVERSAMPLING_16;
		if (HAL_UART_Init(&huart7) != HAL_OK)
		{
				Error_Handler();
		}
		
		__HAL_UART_CLEAR_IDLEFLAG(&huart7);
		__HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);									//使能空闲中断
		UART_Receive_DMA_No_IT(&huart7, USART7_RX_BUF , USART7_MAX_RECV_LEN);
}

uint8_t res = 0;
static void data_solve(nx_receive_msg_t *pc_receive_msg, uint8_t *rx_data)
{
	if(rx_data[SOF_ADDR] == FRAME_HEADER) 
	{	
		//帧尾CRC16校验
			memcpy(pc_receive_msg, rx_data, 29);	
	}
}

//触发空闲中断时进行数据的校验与拷贝
void UART7_IRQHandler(void)
{
	if (__HAL_UART_GET_FLAG(&huart7, UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(&huart7, UART_IT_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart7);
		uint32_t DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart7.hdmarx); 
		USART7_RX_STA = USART7_MAX_RECV_LEN - huart7.hdmarx->Instance->NDTR; 
		__HAL_DMA_DISABLE(huart7.hdmarx);
		data_solve(&pc_receive_msg,USART7_RX_BUF);
		__HAL_DMA_CLEAR_FLAG(huart7.hdmarx, DMA_FLAGS);	
		__HAL_DMA_SET_COUNTER(huart7.hdmarx,USART7_MAX_RECV_LEN);
		__HAL_DMA_ENABLE(huart7.hdmarx);
	}
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
