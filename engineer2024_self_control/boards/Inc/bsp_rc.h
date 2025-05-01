#ifndef BSP_RC_H
#define BSP_RC_H

#include "stdint.h"

/// @brief 遥控器DMA初始化
/// @param rx1_buf 遥控器接收缓存区1
/// @param rx2_buf 遥控器接收缓存区2
/// @param dma_buf_num 遥控器DMA缓存区大小
extern void rc_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

#endif
