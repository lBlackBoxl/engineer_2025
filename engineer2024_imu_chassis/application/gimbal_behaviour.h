#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H
#include "struct_typedef.h"


typedef enum
{
    GIMBAL_MODE_ZERO_FORCE = 0, //无力模式
    GIMBAL_MODE_ABSOLUTE_ANGLE, //陀螺仪模式，用陀螺仪反馈
    GIMBAL_MODE_PC,             //PC辅助瞄准
} gimbal_mode_e;
//通过遥控器设置步兵的工作模式（云台和步兵相同）。针对不同的工作模式存在不同的底盘行为，故工作模式>底盘行为类型，不同的工作模式可以使用相同的底盘行为模式
typedef enum
{
    INFANTRY_MODE_RAW,                   //底盘无力, 跟没上电那样
    INFANTRY_MODE_NO_FOLLOW_YAW,                //底盘不跟随云台角度
    INFANTRY_MODE_FOLLOW_YAW,                   //正常步兵底盘跟随云台
    INFANTRY_MODE_GYRO,                          //小陀螺模式
	  INFANTRY_MODE_PC,
} infantry_mode_e;

extern void gimbal_behaviour_mode_set(void);
extern void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch);
extern bool_t gimbal_cmd_to_shoot_stop(void);
#endif

