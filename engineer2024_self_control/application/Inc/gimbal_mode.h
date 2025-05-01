#ifndef GIMBAL_MODE_H
#define GIMBAL_MODE_H
#include "gimbal_task.h"
//ÊÓ¾õµ÷ÊÔ1 Ò£¿ØÆ÷¿ØÖÆ0
#define RC_DEBUG 0

extern void gimbal_angle_add(float32_t *add_yaw, float32_t *add_pitch, gimbal_control_t *gimbal_control_set);
extern void motor_mode_set(gimbal_control_t *motor_set);
extern void gimbal_absolute_angle_control(float32_t *yaw, float32_t *pitch, gimbal_control_t *gimbal_control_set);
extern void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, float32_t add);
extern void gimbal_relative_angle_control(float32_t *yaw, float32_t *pitch, gimbal_control_t *gimbal_control_set);
extern void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, float32_t add);
extern void motor_zero_force_control(gimbal_motor_t *gimbal_motor);
extern void motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);
extern void motor_relative_angle_control(gimbal_motor_t *gimbal_motor);
#endif

