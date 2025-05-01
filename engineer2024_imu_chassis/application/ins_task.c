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
#include "tim.h"
#include "motor_task.h"
#include "6dof_kinematic.h"
INS_t INS;
IMU_Param_t IMU_Param;
PID_t TempCtrl = {0};

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

uint32_t INS_DWT_Count = 0;
float dt = 0, t = 0;
uint8_t ins_debug_mode = 0;
float RefTemp = 40;

static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);

void INS_Init(void)
{
    IMU_Param.scale[0] = 1;
    IMU_Param.scale[1] = 1;
    IMU_Param.scale[2] = 1;
    IMU_Param.Yaw = 0;
    IMU_Param.Pitch = 0;
    IMU_Param.Roll = 0;
    IMU_Param.flag = 1;

    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0);
    // imu heat init
    PID_Init(&TempCtrl, 2000, 300, 0, 1000, 20, 0, 0, 0, 0, 0, 0, 0);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

    INS.AccelLPF = 0.0085;//不知道这个是啥参数
}

void ins_task(void const * argument)
{
    static uint32_t count = 0;
    const float gravity[3] = {0, 0, 9.81f};
     BMI088_Init(BMI088_MODE);
    INS_Init();
    while(1) 
		{
        dt = DWT_GetDeltaT(&INS_DWT_Count);
        t += dt;

        // ins update
        if ((count % 1) == 0)
        {
            BMI088_Read(&BMI088);

            INS.Accel[0] = BMI088.Accel[0];
            INS.Accel[1] = BMI088.Accel[1];
            INS.Accel[2] = BMI088.Accel[2];
            INS.Gyro[0] = BMI088.Gyro[0];
            INS.Gyro[1] = BMI088.Gyro[1];
            INS.Gyro[2] = BMI088.Gyro[2];
            IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);
            INS.atanxz = -atan2f(INS.Accel[0], INS.Accel[2]) * 180 / PI;
            INS.atanyz = atan2f(INS.Accel[1], INS.Accel[2]) * 180 / PI;

            IMU_QuaternionEKF_Update(INS.Gyro[0], INS.Gyro[1], INS.Gyro[2], INS.Accel[0], INS.Accel[1], INS.Accel[2], dt);

            memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

            BodyFrameToEarthFrame(xb, INS.xn, INS.q);
            BodyFrameToEarthFrame(yb, INS.yn, INS.q);
            BodyFrameToEarthFrame(zb, INS.zn, INS.q);//把机体坐标系下的单位向量转换成了惯性系下的单位向量，不知道有啥用

            float gravity_b[3];
            EarthFrameToBodyFrame(gravity, gravity_b, INS.q);//把惯性系下的重力加速度转化成机体系下的重力加速度

            for (uint8_t i = 0; i < 3; i++)//机体加速度低通滤波
                INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
            BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q);//机体系测的机体加速度换到惯性系里面

            INS.Yaw = QEKF_INS.Yaw;//角度制
            INS.Pitch = QEKF_INS.Pitch;
            INS.Roll = QEKF_INS.Roll;
            INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
						INS.Angle[0] = INS.Yaw/ANGLE_TO_RAD;
						INS.Angle[1] = INS.Pitch/ANGLE_TO_RAD;
						INS.Angle[2] = INS.Roll/ANGLE_TO_RAD;
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
				
				armend.Gyro_pitch = (INS.Pitch)*3.1415926535f/180.0f;
				armend.Gyro_roll = (INS.Roll)*3.1415926535f/180.0f;
				armend.Gyro_yaw = (INS.YawTotalAngle)*3.1415926535f/180.0f;
				
        osDelay(1);
    }
}

/**
 * @brief        Update quaternion（四元数更新，实际没用到）
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
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
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion（欧拉角转换成四元数）
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
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
 * @brief          Transform 3dvector from BodyFrame to EarthFrame（机体坐标系转换为惯性系，四元数版）
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
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
 * @brief          Transform 3dvector from EarthFrame to BodyFrame（惯性系转换到机体系中，四元数版）
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
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

static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])//把角速度和加速度由机体系换到惯性系
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
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
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[0] = c_11 * gyro_temp[0] +
              c_12 * gyro_temp[1] +
              c_13 * gyro_temp[2];
    gyro[1] = c_21 * gyro_temp[0] +
              c_22 * gyro_temp[1] +
              c_23 * gyro_temp[2];
    gyro[2] = c_31 * gyro_temp[0] +
              c_32 * gyro_temp[1] +
              c_33 * gyro_temp[2];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        accel_temp[i] = accel[i];

    accel[0] = c_11 * accel_temp[0] +
               c_12 * accel_temp[1] +
               c_13 * accel_temp[2];
    accel[1] = c_21 * accel_temp[0] +
               c_22 * accel_temp[1] +
               c_23 * accel_temp[2];
    accel[2] = c_31 * accel_temp[0] +
               c_32 * accel_temp[1] +
               c_33 * accel_temp[2];
//坐标系变换,从机体系变换到惯性系中（姿态角表示）
    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

void IMU_Temperature_Ctrl(void)
{
    PID_Calculate(&TempCtrl, BMI088.Temperature, RefTemp);

    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, float_constrain(float_rounding(TempCtrl.Output), 0, UINT32_MAX));
}
/**
  * @brief          获取欧拉角, 0:yaw, 1:pitch, 2:roll 单位 rad
  * @param[in]      none
  * @retval         INS_angle的指针
  */
const fp32 *get_angle_data_point(void)
{
    return INS.Angle;
}

/**
  * @brief          获取角速度,0:x轴, 1:y轴, 2:roll轴 单位 rad/s
  * @param[in]      none
  * @retval         INS_gyro的指针
  */
extern const fp32 *get_gyro_data_point(void)
{
    return INS.Gyro;
}
