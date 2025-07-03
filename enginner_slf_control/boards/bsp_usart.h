#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "usart.h"
#include "stdint.h"

#define USART6_MAX_RECV_LEN 100                     //ros minipc最大缓冲字节数

extern uint8_t USART6_RX_BUF[USART6_MAX_RECV_LEN];  //ros minipc接收缓冲
extern uint16_t USART6_RX_STA;                      //ros minipc当前缓冲长度

extern void usart6_idle_init(void);

#endif
