#ifndef BMI088DRIVER_H
#define BMI088DRIVER_H
#include "struct_typedef.h"
#include "spi.h"

#define BMI088_SPI &hspi1



#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_WRITE_ACCEL_REG_NUM  6   //加速度设置参数列表
#define BMI088_WRITE_GYRO_REG_NUM   6  //陀螺仪设置参数列表


#define BMI088_LONG_DELAY_TIME      80  //复位时间
#define BMI088_COM_WAIT_SENSOR_TIME 150 //等待传感器响应时间


#define BMI088_ACCEL_3G_SEN     0.0008974358974f
#define BMI088_ACCEL_6G_SEN     0.00179443359375f
#define BMI088_ACCEL_12G_SEN    0.0035888671875f
#define BMI088_ACCEL_24G_SEN    0.007177734375f


#define BMI088_GYRO_2000_SEN    0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN    0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN     0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN     0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN     0.000066579027251980956150958662738366f

#define INFANTRY_ID 0
// 陀螺仪校准参数（需手动修改）
#if INFANTRY_ID == 0
#define GxOFFSET  0.00547775626f
#define GyOFFSET -0.0068521807f
#define GzOFFSET  0.00124504545f
#define gNORM 9.7880125f
#elif INFANTRY_ID == 1
#define GxOFFSET 0.0007222f
#define GyOFFSET -0.001786f
#define GzOFFSET 0.0004346f
#define gNORM 9.876785f
#elif INFANTRY_ID == 2
#define GxOFFSET 0.0007222f
#define GyOFFSET -0.001786f
#define GzOFFSET 0.0004346f
#define gNORM 9.876785f
#elif INFANTRY_ID == 3
#define GxOFFSET 0.00270364084f
#define GyOFFSET -0.000532632112f
#define GzOFFSET 0.00478090625f
#define gNORM 9.73574924f
#elif INFANTRY_ID == 4
#define GxOFFSET 0.0007222f
#define GyOFFSET -0.001786f
#define GzOFFSET 0.0004346f
#define gNORM 9.876785f
#endif

typedef struct
{
    float Accel[3];

    float Gyro[3];

    float TempWhenCali;
    float Temperature;

    float AccelScale;
    float GyroOffset[3];

    float gNorm;
} IMU_Data_t;

enum
{
    BMI088_NO_ERROR                     = 0x00,
    BMI088_ACC_PWR_CTRL_ERROR           = 0x01,
    BMI088_ACC_PWR_CONF_ERROR           = 0x02,
    BMI088_ACC_CONF_ERROR               = 0x03,
    BMI088_ACC_SELF_TEST_ERROR          = 0x04,
    BMI088_ACC_RANGE_ERROR              = 0x05,
    BMI088_INT1_IO_CTRL_ERROR           = 0x06,
    BMI088_INT_MAP_DATA_ERROR           = 0x07,
    BMI088_GYRO_RANGE_ERROR             = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR         = 0x09,
    BMI088_GYRO_LPM1_ERROR              = 0x0A,
    BMI088_GYRO_CTRL_ERROR              = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR  = 0x0D,

    BMI088_SELF_TEST_ACCEL_ERROR        = 0x80,
    BMI088_SELF_TEST_GYRO_ERROR         = 0x40,
    BMI088_NO_SENSOR                    = 0xFF,
};



extern bool_t bmi088_accel_self_test(void);//加速度计自检
extern bool_t bmi088_accel_init(void); //加速度初始化
extern bool_t bmi088_gyro_self_test(void); //陀螺仪自检
extern bool_t bmi088_gyro_init(void);  //陀螺仪初始化
extern void Calibrate_MPU_Offset(IMU_Data_t *bmi088); //陀螺仪校准
extern uint8_t BMI088_init(uint8_t calibrate);
extern void BMI088_Init(uint8_t BMI088_mode);

extern void BMI088_accel_read_over(uint8_t *rx_buf, fp32 accel[3], fp32 *time);
extern void BMI088_gyro_read_over(uint8_t *rx_buf, fp32 gyro[3]);
extern void BMI088_temperature_read_over(uint8_t *rx_buf, fp32 *temperate);
extern void BMI088_Read(IMU_Data_t *bmi088);
extern uint32_t get_BMI088_sensor_time(void);
extern fp32 get_BMI088_temperate(void);
extern void get_BMI088_gyro(int16_t gyro[3]);
extern void get_BMI088_accel(fp32 accel[3]);

extern void BMI088_read_gyro_who_am_i(void);
extern void BMI088_read_accel_who_am_i(void);

extern void imu_pwm_set(uint16_t pwm); //温度控制
extern IMU_Data_t BMI088;
#endif

