#ifndef BSP_DELAY_H
#define BSP_DELAY_H

#include "stdint.h"

/// @brief 系统延时初始化
/// @param  无
extern void delay_init(void);

/// @brief 微秒级延时
/// @param nus 延时周期
extern void delay_us(uint16_t nus);

/// @brief 毫秒级延时
/// @param nms 延时周期
extern void delay_ms(uint16_t nms);

#endif
