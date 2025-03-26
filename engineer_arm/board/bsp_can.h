#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stdint.h"

#define RC_CAN hcan2 

/// @brief CAN通信过滤器初始化
/// @param 无
extern void can_filter_init(void);

#endif
