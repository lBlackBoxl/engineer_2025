#ifndef IMU_TASK_H
#define IMU_TASK_H

#include "bsp_imu.h"

extern void imu_task(void const *pvParameters);

extern imu_t              imu;

#endif //IMU_TASK_H