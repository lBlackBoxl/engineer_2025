#ifndef BSP_REFEREE_H
#define BSP_REFEREE_H

#include <string.h>
#include "struct_typedef.h"

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
    float data[6];
		uint16_t time_stamp;
		uint32_t  nothing;
    uint16_t crc16;
}__attribute__((packed)) tFrame;


#ifndef REFEREE_HUART
#define REFEREE_HUART huart6
#endif

extern uint8_t referee_tx_send(fp32 motor_data[7]);

#endif
