#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "CAN_receive.h"


//任务初始化延时
#define GIMBAL_TASK_INIT_TIME	500
//云台任务的周期（1ms）
#define GIMBAL_CONTROL_TIME	1
#define BMI088_ROLL_TURN 0
extern void gimbal_task(void const *pvParameters);
#endif

