#ifndef IMU_TASK_H
#define IMU_TASK_H

#include "stdint.h"
#include "arm_math.h"

#define SPI_DMA_GYRO_LENGHT       8
#define SPI_DMA_ACCEL_LENGHT      9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4
#define IMU_DR_SHFITS        0
#define IMU_SPI_SHFITS       1
#define IMU_UPDATE_SHFITS    2
#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2
#define TEMPERATURE_PID_KP 1600.0f          //温度控制PID的kp
#define TEMPERATURE_PID_KI 0.2f             //温度控制PID的ki
#define TEMPERATURE_PID_KD 0.0f             //温度控制PID的kd
#define TEMPERATURE_PID_MAX_OUT 4500.0f     //温度控制PID的max_out
#define TEMPERATURE_PID_MAX_IOUT 4400.0f    //温度控制PID的max_iout
#define MPU6500_TEMP_PWM_MAX 5000           //mpu6500控温TIM的重载值
#define IMU_TASK_INIT_TIME 7                //任务开始延时
#define IMU_YAW_ADDRESS_OFFSET    0
#define IMU_PITCH_ADDRESS_OFFSET  1
#define IMU_ROLL_ADDRESS_OFFSET   2
#define IMU_GYRO_X_ADDRESS_OFFSET 0
#define IMU_GYRO_Y_ADDRESS_OFFSET 1
#define IMU_GYRO_Z_ADDRESS_OFFSET 2
#define IMU_ACCEL_X_ADDRESS_OFFSET 0
#define IMU_ACCEL_Y_ADDRESS_OFFSET 1
#define IMU_ACCEL_Z_ADDRESS_OFFSET 2
#define IMU_MAG_X_ADDRESS_OFFSET 0
#define IMU_MAG_Y_ADDRESS_OFFSET 1
#define IMU_MAG_Z_ADDRESS_OFFSET 2

#define CALI_MODE 0          //校准模式下上电后进行校准，否则上电后不校准
#define START_CALI_TIME 5000    //开始校准时间，单位：ms
#define MAX_CALI_TIME 30000     //最大校准时间，单位：ms

typedef enum
{
    CALI_ON = 1, 	//陀螺仪校准中
    CALI_FINISH,   	//陀螺仪校准完毕
} cali_mode_e;
extern float32_t IMU_angle[3];

extern void imu_task(void const *pvParameters);
extern const float32_t *get_imu_quat_point(void);
extern const float32_t *get_imu_angle_point(void);
extern const float32_t *get_gyro_data_point(void);
extern const float32_t *get_accel_data_point(void);

#endif

