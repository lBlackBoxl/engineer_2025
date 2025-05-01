#ifndef LED_TASK_H
#define LED_TASK_H

#include "stdint.h"
#include "arm_math.h"

#define RGB_PERIOD  1000    //流水灯变换周期(毫秒)
#define RGB_COLOR   6       //流水灯颜色种类

/// @brief 流水灯任务
/// @param argument 未使用
extern void led_task(void const * argument);

#endif
