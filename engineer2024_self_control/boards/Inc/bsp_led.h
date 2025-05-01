#ifndef BSP_LED_H
#define BSP_LED_H

#include "stdint.h"

/// @brief RGB灯光
/// @param argb LED的RGB值及透明度
extern void led_argb(uint32_t argb);

#endif

