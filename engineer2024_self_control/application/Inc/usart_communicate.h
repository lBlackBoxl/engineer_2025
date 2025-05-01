#ifndef USART_COMMUNICATE_H
#define USART_COMMUNICATE_H

#include "stdint.h"
#include "CRC.h"

typedef struct
{
    uint8_t  sof;
    uint16_t dataLenth;
    uint8_t  seq;
    uint8_t  crc8;
}__attribute__((packed)) tFrameHeader;

typedef struct
{
    tFrameHeader header;
    uint16_t cmd_id;
    float data[7];
    uint8_t unused_data[2];
    uint16_t crc16;
}__attribute__((packed)) send_data;

//typedef struct
//{
//	uint8_t motor_position_data[12];
//	uint8_t qua_data[12];
//}tx_self_control_message;

extern send_data send_data_self_control;
void usart_communicate_task(void const * argument);

#endif
