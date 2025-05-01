#ifndef DETECT_TASK_H
#define DETECT_TASK_H

#include "stdint.h"

#define DETECT_TASK_INIT_TIME 57
#define DETECT_CONTROL_TIME 50

//错误码以及对应设备顺序
enum errorList
{
	DBUS_TOE = 0,
    GIMBAL_FRIC1_TOE,
    GIMBAL_FRIC2_TOE,
	GIMBAL_TRIGGER_TOE,
    GIMBAL_PIT_TOE,
    GIMBAL_YAW_TOE,
};
typedef struct
{
	uint64_t error_time;
    uint32_t new_time;
    uint16_t set_offline_time : 12;
    uint8_t error_exist : 1;
} error_t;

//检测任务
extern void detect_task(void const *pvParameters);
//记录时间
extern void detect_hook(uint8_t toe);
//获取错误列表
extern const error_t *get_error_list_point(void);
#endif
