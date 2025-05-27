#ifndef BSP_UART_H
#define BSP_UART_H

#include "struct_typedef.h"
#include "fifo.h"
#include "usart.h"

#define USART_RX_BUF_LEN 100

extern fifo_s_t referee_fifo;

extern void uart1_init(void);
extern void uart6_init(void);

extern void usart1_IRQ_Callback(void);
extern void usart6_IRQ_Callback(void);

#endif


