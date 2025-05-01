#ifndef AHRS_H
#define AHRS_H

#include "stdint.h"
#include "arm_math.h"

#define FS	1000.0f			//惯性测量元件采样率
#define KP2	(2.0f * 0.5f)	//AHRS算法的PI控制器系数
#define KI2	(2.0f * 0.0f)	//AHRS算法的PI控制器系数

/// @brief IMU初始化
/// @param quat 四元数
/// @param accel 三轴加速度计数据
/// @param mag 三轴磁力计数据
extern void AHRS_init(float32_t quat[4]);

/// @brief IMU数据更新
/// @param quat 四元数
/// @param gyro 三轴角速度计数据
/// @param accel 三轴加速度计数据
/// @param mag 三轴磁力计数据
extern void AHRS_update(float32_t quat[4], float32_t gyro[3], float32_t accel[3]);

/// @brief 获取IMU角度
/// @param q 四元数
/// @param yaw yaw轴角度
/// @param pitch pitch轴角度
/// @param roll roll轴角度
extern void AHRS_get(float32_t q[4], float32_t *yaw, float32_t *pitch, float32_t *roll);

/// @brief AHRS算法
/// @param q 四元数
/// @param gx 绕x轴角速度
/// @param gy 绕y轴角速度
/// @param gz 绕z轴角速度
/// @param ax x轴加速度
/// @param ay y轴加速度
/// @param az z轴加速度
extern void AHRS_calc(float32_t q[4], float32_t gx, float32_t gy, float32_t gz, float32_t ax, float32_t ay, float32_t az);

#endif
