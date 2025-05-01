#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include "CAN_receive.h"

typedef struct
{
	const motor_DM_motor_t *DM_motor_measure;
	uint16_t round_cnt;
	fp32 total_angle;
	fp32 offset_angle;
	fp32 position_set;
	fp32 speed_set;
}motor_DM_t;

typedef struct
{
	motor_DM_t motor_DM_data[3];
	fp32 Gyro_pitch;
	fp32 Gyro_roll;
	fp32 Gyro_yaw;
}self_control_t;

extern fp32 self_control_motor_position[3];
extern self_control_t armend;
extern void motor_task(void const * argument);

#endif

