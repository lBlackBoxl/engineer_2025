#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "usart.h"
#include "main.h"
//最大缓冲字节数
#define USART6_MAX_RECV_LEN 100
//接收缓冲


extern void uart6_init(void);
extern void uart1_init(void);

#endif


