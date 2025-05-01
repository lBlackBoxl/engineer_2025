/**
 ******************************************************************************
 * @file    ins_task.c
 * @author  Wang Hongxi
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "ins_task.h"
#include "controller.h"
#include "QuaternionEKF.h"
#include "bsp_PWM.h"
#include "BMI088driver.h"
#include "struct_typedef.h"
#include "arm_math.h"

INS_t INS;
IMU_Param_t IMU_Param;
PID_t TempCtrl = {0};

const fp32 xb[3] = {1, 0, 0};
const fp32 yb[3] = {0, 1, 0};
const fp32 zb[3] = {0, 0, 1};
fp32 INS_angle[3] = {0, 0, 0};
vector_calc_t vector;

uint32_t INS_DWT_Count = 0;
fp32 dt = 0, t = 0;
uint8_t ins_debug_mode = 0;
fp32 RefTemp = 40;
fp32 last_gyro[3];
fp32 last_accel[3];
Ordinary_Least_Squares_t accel[3];
Butter_worth_Filter_t   accelb[3];

static void cross_product(const vector3_t *a, const vector3_t *b, vector3_t *c);
static void IMU_Param_Correction(IMU_Param_t *param, fp32 gyro[3], fp32 accel[3]);


uint32_t more_ins;
void INS_Init(void)
{

    IMU_Param.scale[X] = 1;
    IMU_Param.scale[Y] = 1;
    IMU_Param.scale[Z] = 1;
    IMU_Param.Yaw = 0;
    IMU_Param.Pitch = 0;
    IMU_Param.Roll = 0;
    IMU_Param.flag = 1;
    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0);
	
	
	
    // imu heat init
    PID_Init(&TempCtrl, 2000, 300, 0, 1000, 20, 0, 0, 0, 0, 0, 0, 0);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

    INS.AccelLPF = 0.0085;
	  
	dt = DWT_GetDeltaT(&INS_DWT_Count);
	
	Butter_worth_Filter_init(&accelb[0], 100, dt);
	Butter_worth_Filter_init(&accelb[1], 100, dt);
	Butter_worth_Filter_init(&accelb[2], 100, dt);

//	OLS_Init(&accel[0],5);		
//	OLS_Init(&accel[1],5);
//	OLS_Init(&accel[2],5);

}

void ins_Task(void)
{

    static uint32_t count = 0;
    const fp32 gravity[3] = {0, 0, 9.81f};
    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;

    // ins update
    if ((count % 1) == 0)
    {
        BMI088_Read(&BMI088);
		//相对于车体坐标系的角度 与加速度
        INS.Accel[X] = BMI088.Accel[X];
        INS.Accel[Y] = BMI088.Accel[Y];
        INS.Accel[Z] = BMI088.Accel[Z];
        INS.Gyro[X] = BMI088.Gyro[X];
        INS.Gyro[Y] = BMI088.Gyro[Y];
        INS.Gyro[Z] = BMI088.Gyro[Z];
		

        IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);
        INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]) * 180 / PI;
        INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]) * 180 / PI;

        IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);

        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
        BodyFrameToEarthFrame(zb, INS.zn, INS.q);

        fp32 gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        for (uint8_t i = 0; i < 3; i++)
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
       
	   BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q);

        INS.Yaw = QEKF_INS.Yaw;
        INS.Pitch = QEKF_INS.Pitch;
        INS.Roll = QEKF_INS.Roll;
        INS.YawTotalAngle = QEKF_INS.YawTotalAngle;

		//相对于世界坐标系的角度
		INS_angle[0] = INS.Yaw/57.295779513f;
		INS_angle[1] = INS.Pitch/57.295779513f;
		INS_angle[2] = INS.Roll/57.295779513f;
		
		//INS_angle[] 1pitch   0yaw  2roll
		//Wx roll Wy yaw Wz pitch
		//INS.Gyro[]  0pitch   2yaw  1roll
		//相对于世界坐标系的角速度与角加速度 1y  0z 2x  
	
		INS.ang_gyro[X] = (arm_cos_f32(ROLL)*arm_cos_f32(YAW))*INS.Gyro[1] + (arm_cos_f32(ROLL)*arm_sin_f32(PITCH)*arm_sin_f32(YAW) - arm_cos_f32(PITCH)*arm_sin_f32(ROLL))*INS.Gyro[2] 
		+ (arm_sin_f32(PITCH)*arm_sin_f32(ROLL) + arm_cos_f32(PITCH)*arm_cos_f32(ROLL)*arm_sin_f32(YAW))*INS.Gyro[0];
		INS.ang_gyro[Y] = (arm_cos_f32(YAW)*arm_sin_f32(ROLL))*INS.Gyro[1] + (arm_cos_f32(PITCH)*arm_cos_f32(ROLL) + arm_sin_f32(PITCH)*arm_sin_f32(ROLL)*arm_sin_f32(YAW))*INS.Gyro[2] 
		+ (arm_cos_f32(PITCH)*arm_sin_f32(ROLL)*arm_sin_f32(YAW) - arm_cos_f32(ROLL)*arm_sin_f32(PITCH))*INS.Gyro[0];
		INS.ang_gyro[Z] = (-arm_sin_f32(YAW))*INS.Gyro[1] + (arm_cos_f32(YAW)*arm_sin_f32(PITCH))*INS.Gyro[2] + (arm_cos_f32(PITCH)*arm_cos_f32(YAW))*INS.Gyro[0];
		
//		INS.ang_accel[X] = OLS_Derivative(&accel[0],dt,INS.ang_gyro[X]);
//		INS.ang_accel[Y] = OLS_Derivative(&accel[1],dt,INS.ang_gyro[Y]);
//		INS.ang_accel[Z] = OLS_Derivative(&accel[2],dt,INS.ang_gyro[Z]);
		
 			last_accel[0] = (INS.ang_gyro[0]-last_gyro[0]);
 			last_accel[1] = (INS.ang_gyro[1]-last_gyro[1]);
 			last_accel[2] = (INS.ang_gyro[2]-last_gyro[2]);
					
		INS.ang_accelb[X] = Butter_worth_Filter_calc(&accelb[0],last_accel[0]);
		INS.ang_accelb[Y] = Butter_worth_Filter_calc(&accelb[1],last_accel[1]);
		INS.ang_accelb[Z] = Butter_worth_Filter_calc(&accelb[2],last_accel[2]);
		last_gyro[X] = INS.ang_gyro[X];
		last_gyro[Y] = INS.ang_gyro[Y];
		last_gyro[Z] = INS.ang_gyro[Z];
	
 
		vector.vector_dW->x = INS.ang_accelb[X];
		vector.vector_dW->y = INS.ang_accelb[Y];
		vector.vector_dW->z = INS.ang_accelb[Z];
		vector.vector_W->x = INS.ang_gyro[X];
		vector.vector_W->y = INS.ang_gyro[Y];
		vector.vector_W->z = INS.ang_gyro[Z];
		vector.vector_Rpa->x = RX;
		vector.vector_Rpa->y = RY;
		vector.vector_Rpa->z = RZ;

		cross_product(vector.vector_dW,vector.vector_Rpa,vector.vector_1);
		cross_product(vector.vector_W,vector.vector_Rpa,vector.vector_2);
		cross_product(vector.vector_W,vector.vector_2,vector.vector_3);
	
		// xyz 加速度 0 2 1 角度
		INS.BodyAccel_n[0] = INS.MotionAccel_n[0] + vector.vector_1->x +vector.vector_3->x;							
		INS.BodyAccel_n[2] = INS.MotionAccel_n[2] + vector.vector_1->y +vector.vector_3->y;
		INS.BodyAccel_n[1] = -INS.MotionAccel_n[1] + vector.vector_1->z +vector.vector_3->z;							
		
    }

    // temperature control
    if ((count % 2) == 0)
    {
        // 500hz
        IMU_Temperature_Ctrl();
    }

    if ((count % 1000) == 0)
    {
        // 200hz
    }

    count++;

//		more_ins  = uxTaskGetStackHighWaterMark(NULL);
	
}

/**
 * @brief        Update quaternion
 */
void QuaternionUpdate(fp32 *q, fp32 gx, fp32 gy, fp32 gz, fp32 dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(fp32 *q, fp32 *Yaw, fp32 *Pitch, fp32 *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(fp32 Yaw, fp32 Pitch, fp32 Roll, fp32 *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const fp32 *vecBF, fp32 *vecEF, fp32 *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const fp32 *vecEF, fp32 *vecBF, fp32 *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

static void IMU_Param_Correction(IMU_Param_t *param, fp32 gyro[3], fp32 accel[3])
{
    static fp32 lastYawOffset, lastPitchOffset, lastRollOffset;
    static fp32 c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    fp32 gyro_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[X] = c_11 * gyro_temp[X] +
              c_12 * gyro_temp[Y] +
              c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] +
              c_22 * gyro_temp[Y] +
              c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] +
              c_32 * gyro_temp[Y] +
              c_33 * gyro_temp[Z];

    fp32 accel_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        accel_temp[i] = accel[i];

    accel[X] = c_11 * accel_temp[X] +
               c_12 * accel_temp[Y] +
               c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] +
               c_22 * accel_temp[Y] +
               c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] +
               c_32 * accel_temp[Y] +
               c_33 * accel_temp[Z];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

extern const fp32 *get_gyro_data_point(void)
{
    return INS.Gyro;
}
extern const fp32 *get_accel_data_point(void)
{
	
    return INS.BodyAccel_n;
}
const fp32 *get_angle_data_point(void)
{
		return INS_angle;
}

void IMU_Temperature_Ctrl(void)
{
    PID_Calculate(&TempCtrl, BMI088.Temperature, RefTemp);

    TIM_Set_PWM(&htim10, TIM_CHANNEL_1, fp32_constrain(fp32_rounding(TempCtrl.Output), 0, UINT32_MAX));
}
//叉乘
void cross_product(const vector3_t *a, const vector3_t *b, vector3_t *c) 
{
    c->x = a->y * b->z - a->z * b->y;
    c->y = a->z * b->x - a->x * b->z;
    c->z = a->x * b->y - a->y * b->x;
}

