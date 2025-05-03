#include "referee_usart_task.h"
#include "bsp_usart.h"
#include "protocol.h"
#include "string.h"
#include "usart.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;

uint8_t  USART2_RX_BUF[USART2_MAX_RECV_LEN]; 
uint16_t USART2_RX_STA = 0;
uint8_t  USART6_RX_BUF[2][USART_RX_BUF_LENGHT];
receive_msg_t imu_rx_data;

//和算法通信的串口初始化
//void usart2_idle_init(void)
//{
//	__HAL_UART_CLEAR_IDLEFLAG(&huart2);
//	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
//	UART_Receive_DMA_No_IT(&huart2, USART2_RX_BUF , USART2_MAX_RECV_LEN);
//}

void usart2_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //使能DMA串口接收和发送
    SET_BIT(huart2.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart2.Instance->CR3, USART_CR3_DMAT);
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart2_rx);
    while(hdma_usart2_rx.Instance->CR & DMA_SxCR_EN)
        __HAL_DMA_DISABLE(&hdma_usart2_rx);
    __HAL_DMA_CLEAR_FLAG(&hdma_usart2_rx, DMA_LISR_TCIF1);
    hdma_usart2_rx.Instance->PAR = (uint32_t) & (USART2->DR);
    //内存缓冲区1
    hdma_usart2_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //内存缓冲区2
    hdma_usart2_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_usart2_rx, dma_buf_num);
    //使能双缓冲区
    SET_BIT(hdma_usart2_rx.Instance->CR, DMA_SxCR_DBM);
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart2_rx);
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart2_tx);
    while(hdma_usart2_tx.Instance->CR & DMA_SxCR_EN)
        __HAL_DMA_DISABLE(&hdma_usart2_tx);
    hdma_usart2_tx.Instance->PAR = (uint32_t) & (USART2->DR);
}


	
void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //使能DMA串口接收和发送
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);
    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //内存缓冲区1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //内存缓冲区2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);
    //使能双缓冲区
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);
    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);
}

void uart5_init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart5);
	__HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
}

void uart6_init(void)
{
	  usart6_init(USART6_RX_BUF[0], USART6_RX_BUF[1], USART_RX_BUF_LENGHT);
}

void uart7_init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart7);
  __HAL_UART_ENABLE_IT(&huart7, UART_IT_RXNE);  //receive interrupt
	//__HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);  //receive interrupt
}

void uart8_init(void)
{
  //open uart idle it	
  	//开启空闲中断
  __HAL_UART_CLEAR_IDLEFLAG(&huart8);
  __HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);  //receive interrupt
}

void USART6_IRQHandler(void)
{
    static volatile uint8_t res;
    if(USART6->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);
        static uint16_t this_time_rx_len = 0;
        if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart6.hdmarx);
						if (verify_crc16_check_sum(USART6_RX_BUF[0],sizeof(receive_msg_t)))
						{
								memcpy(&imu_rx_data, USART6_RX_BUF[0], sizeof(receive_msg_t));
						}
						
        }
        else
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart6.hdmarx);
						if (verify_crc16_check_sum(USART6_RX_BUF[1],sizeof(receive_msg_t)))
						{
								memcpy(&imu_rx_data, USART6_RX_BUF[1], sizeof(receive_msg_t));
						}	
        }
    }
}
