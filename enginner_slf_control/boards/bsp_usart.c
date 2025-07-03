#include "bsp_usart.h"
#include "usart.h"
#include "stdio.h"

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;

uint8_t  USART6_RX_BUF[USART6_MAX_RECV_LEN]; 
uint16_t USART6_RX_STA = 0;

static int UART_Receive_DMA_No_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);

void usart6_idle_init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart6);
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
	UART_Receive_DMA_No_IT(&huart6, USART6_RX_BUF , USART6_MAX_RECV_LEN);
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
