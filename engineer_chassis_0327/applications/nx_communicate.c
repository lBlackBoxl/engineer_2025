#include "FreeRTOS.h"
#include "main.h"
#include "usart.h"
#include "nx_communicate.h"
#include "cmsis_os.h"
#include "string.h"
#include "arm_control_task.h"
#include "boards_communicate.h"

extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;

float txData[6];
float rxData[6];
uint8_t txData_uint[24] = {0};
uint8_t rxData_uint[24] = {0};

extern void nx_communicate_init(void);

void nx_communicate_task(void const *pvParameters)
{
	vTaskDelay(500);
	
	nx_communicate_init();
	
	while(1)
	{   
		txData[0] = arm_control.motor_YAW_data.DM_motor_measure->motor_position;
		txData[1] = arm_message.target_position[1];
		txData[2] = arm_message.target_position[2];
		txData[3] = arm_message.target_position[3];
		txData[4] = arm_message.target_position[4];
		txData[5] = arm_message.target_position[5];
		
		memcpy(txData_uint, txData, sizeof(txData));
		
		HAL_UART_Transmit_DMA(&huart7,txData_uint,sizeof(txData_uint));
		osDelay(10);
		HAL_UART_Receive_DMA(&huart7,rxData_uint,sizeof(rxData_uint));
		osDelay(10);
		
		memcpy(rxData, rxData_uint, sizeof(rxData_uint));
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
	
	__HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);									//使能空闲中断
	
	HAL_UART_Receive_DMA(&huart7, rxData_uint,sizeof(rxData_uint));        //初始化接收中断
	
}

void UART7_IRQHandler(void)
{
	if((__HAL_UART_GET_FLAG(&huart7, UART_FLAG_IDLE)) != RESET)   //进入空闲中断后对数据及DMA通道进行处理
	{
			__HAL_DMA_DISABLE(&hdma_uart7_rx);                        //关闭DMA接收通讯，防止信息传输错误
		
			__HAL_UART_CLEAR_IDLEFLAG(&huart7);                       //清除空闲中断标志位
		
			__HAL_DMA_CLEAR_FLAG(&hdma_uart7_rx,DMA_FLAG_TCIF3_7);		//清除DMA通道标志位
		
			__HAL_DMA_ENABLE(&hdma_uart7_rx);													//使能DMA接收通道
	}
	
  HAL_UART_IRQHandler(&huart7);

}
