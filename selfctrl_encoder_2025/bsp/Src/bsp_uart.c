#include "bsp_uart.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include "bsp_encoder.h"


extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

uint8_t  USART1_RX_BUF[2][USART_RX_BUF_LEN];
uint8_t  USART6_RX_BUF[2][USART_RX_BUF_LEN];

fifo_s_t referee_fifo;

void usart1_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //使能DMA串口接收和发送
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, DMA_LISR_TCIF1);
    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
    //内存缓冲区1
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //内存缓冲区2
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_usart1_rx, dma_buf_num);
    //使能双缓冲区
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);
    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
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


void uart1_init(void)
{
	  usart1_init(USART1_RX_BUF[0], USART1_RX_BUF[1], USART_RX_BUF_LEN);
}

void uart6_init(void)
{
	  usart6_init(USART6_RX_BUF[0], USART6_RX_BUF[1], USART_RX_BUF_LEN);
}

void usart1_IRQ_Callback(void)
{
    static volatile uint8_t res;
    if(USART1->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
        if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, USART_RX_BUF_LEN);
            huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart1.hdmarx);
						unpack_encoder_data(USART1_RX_BUF[0]);
						
        }
        else
        {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, USART_RX_BUF_LEN);
            huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart1.hdmarx);
						unpack_encoder_data(USART1_RX_BUF[1]);

        }
    }
}

void usart6_IRQ_Callback(void)
{
    static volatile uint8_t res;
		static uint16_t this_time_rx_len = 0;
    if(huart6.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);
    }
    else if(USART6->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);
        if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
						this_time_rx_len = USART_RX_BUF_LEN - hdma_usart6_rx.Instance->NDTR;
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LEN);
            huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart6.hdmarx);
						fifo_s_puts(&referee_fifo, (char*)USART6_RX_BUF[0], this_time_rx_len);
        }
        else
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
						this_time_rx_len = USART_RX_BUF_LEN - hdma_usart6_rx.Instance->NDTR;
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LEN);
            huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart6.hdmarx);
						fifo_s_puts(&referee_fifo, (char*)USART6_RX_BUF[0], this_time_rx_len);
        }
    }
}
