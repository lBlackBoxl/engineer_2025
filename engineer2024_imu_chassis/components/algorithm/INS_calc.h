#ifndef __INS_CALC_H
#define __INS_CALC_H
#include "struct_typedef.h"
extern void INS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3]);
extern void INS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3]);
extern void data_update(float q[4], float gx, float gy, float gz, float ax, float ay, float az);
extern void angle_get(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll);
extern float INS_sqrt(float x);
#endif
