#include "bsp_usart.h"
#include "usart.h"

void uart6_init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart6);
//  __HAL_UART_ENABLE_IT(&huart6, UART_ITRXNE)_;  //receive interrupt
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);  
}


void uart1_init(void)
{
//	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
//  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);  //receive interrupt
	//__HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);  
	
		//串口1
  //开启清空标志位
  __HAL_UART_CLEAR_IDLEFLAG(&huart1);
	//开启空闲中断
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}
