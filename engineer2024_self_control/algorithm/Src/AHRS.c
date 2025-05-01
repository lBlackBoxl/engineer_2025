#include "AHRS.h"
#include "math.h"
#include "MATH_LIB.h"

float32_t kp2 = KP2;
float32_t ki2 = KI2;
float32_t ifbx = 0.0f,  ifby = 0.0f, ifbz = 0.0f;

void AHRS_calc(float32_t q[4], float32_t gx, float32_t gy, float32_t gz, float32_t ax, float32_t ay, float32_t az) 
{
	float32_t norm;
	float32_t halfvx, halfvy, halfvz;
	float32_t halfex, halfey, halfez;
	float32_t qa, qb, qc;
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
	{
		norm = quick_sqrt(ax * ax + ay * ay + az * az);
		ax *= norm;
		ay *= norm;
		az *= norm;       
		halfvx = q[1] * q[3] - q[0] * q[2];
		halfvy = q[0] * q[1] + q[2] * q[3];
		halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);
		if(ki2 > 0.0f) 
		{
			ifbx += ki2 * halfex * (1.0f / FS);
			ifby += ki2 * halfey * (1.0f / FS);
			ifbz += ki2 * halfez * (1.0f / FS);
			gx += ifbx;
			gy += ifby;
			gz += ifbz;
		}
		else 
		{
			ifbx = 0.0f;
			ifby = 0.0f;
			ifbz = 0.0f;
		}
		gx += kp2 * halfex;
		gy += kp2 * halfey;
		gz += kp2 * halfez;
	}
	gx *= (0.5f * (1.0f / FS));
	gy *= (0.5f * (1.0f / FS));
	gz *= (0.5f * (1.0f / FS));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx); 
	norm = quick_sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= norm;
	q[1] *= norm;
	q[2] *= norm;
	q[3] *= norm;
}

void AHRS_init(float32_t quat[4])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;
}

void AHRS_update(float32_t quat[4], float32_t gyro[3], float32_t accel[3])
{
    AHRS_calc(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
}

void AHRS_get(float32_t q[4], float32_t *yaw, float32_t *pitch, float32_t *roll)
{
    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
} 
