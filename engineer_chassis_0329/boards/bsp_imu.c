/**
  ***************************************(C) COPYRIGHT 2018 DJI***************************************
  * @file       bsp_imu.c
  * @brief      mpu6500模块驱动，配置MPU6500并通过SPI接口读取加速度计和陀螺仪数据      
  * @note         
  * @Version    V1.0.0
  * @Date       Jan-30-2018      
  ***************************************(C) COPYRIGHT 2018 DJI***************************************
  */

#include "bsp_imu.h"
#include "ist8310_reg.h" 
#include "stm32f4xx_hal.h"
#include <math.h>
#include "mpu6500_reg.h"
#include "spi.h"

#define BOARD_DOWN (0)   // 板子朝下
#define IST8310    // 使用IST8310磁力计
#define MPU_HSPI hspi5  // MPU使用的SPI接口
#define MPU_NSS_LOW HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)  // 拉低NSS引脚
#define MPU_NSS_HIGH HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)   // 拉高NSS引脚

#define Kp 2.0f  // 比例增益，控制加速度计/磁力计的收敛速度
#define Ki 0.01f // 积分增益，控制陀螺仪偏差的收敛速度

volatile float        q0 = 1.0f;  // 四元数q0
volatile float        q1 = 0.0f;  // 四元数q1
volatile float        q2 = 0.0f;  // 四元数q2
volatile float        q3 = 0.0f;  // 四元数q3
volatile float        exInt, eyInt, ezInt;  // 误差积分
static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;  // 陀螺仪、加速度计、磁力计数据
volatile uint32_t     last_update, now_update;  // 采样周期计数，单位为ms
static uint8_t        tx, rx;  // SPI发送和接收的临时变量
static uint8_t        tx_buff[14] = { 0xff };  // 发送缓冲区
uint8_t               mpu_buff[14];  // 保存IMU原始数据的缓冲区
uint8_t               ist_buff[6];   // 保存IST8310原始数据的缓冲区
mpu_data_t            mpu_data;  // MPU数据结构
imu_t                 imu={0};  // IMU数据结构

/**
  * @brief  快速计算1/Sqrt(x)
  * @param  x: 需要计算的数
  * @retval 1/Sqrt(x)
  * @usage  在imu_ahrs_update()函数中调用
  */
float inv_sqrt(float x) 
{
    float halfx = 0.5f * x;
    float y     = x;
    long  i     = *(long*)&y;
    
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    
    return y;
}

/**
  * @brief  向指定寄存器写入一个字节的数据
  * @param  reg:  要写入的寄存器地址
  *         data: 要写入的数据
  * @retval 
  * @usage  在ist_reg_write_by_mpu(),         
  *                 ist_reg_read_by_mpu(), 
  *                 mpu_master_i2c_auto_read_config(), 
  *                 ist8310_init(), 
  *                 mpu_set_gyro_fsr(),             
  *                 mpu_set_accel_fsr(), 
  *                 mpu_device_init() 函数中调用
  */
uint8_t mpu_write_byte(uint8_t const reg, uint8_t const data)
{
    MPU_NSS_LOW;
    tx = reg & 0x7F;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    tx = data;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    MPU_NSS_HIGH;
    return 0;
}

/**
  * @brief  从指定寄存器读取一个字节的数据
  * @param  reg: 要读取的寄存器地址
  * @retval 
  * @usage  在ist_reg_read_by_mpu(),         
  *                 mpu_device_init() 函数中调用
  */
uint8_t mpu_read_byte(uint8_t const reg)
{
    MPU_NSS_LOW;
    tx = reg | 0x80;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    MPU_NSS_HIGH;
    return rx;
}

/**
  * @brief  从指定寄存器读取多个字节的数据
  * @param  regAddr: 要读取的寄存器地址
  * @retval 
  * @usage  在ist8310_get_data(),         
  *                 mpu_get_data(), 
  *                 mpu_offset_call() 函数中调用
  */
uint8_t mpu_read_bytes(uint8_t const regAddr, uint8_t* pData, uint8_t len)
{
    MPU_NSS_LOW;
    tx         = regAddr | 0x80;
    tx_buff[0] = tx;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(&MPU_HSPI, tx_buff, pData, len, 55);
    MPU_NSS_HIGH;
    return 0;
}

/**
  * @brief  通过MPU6500的I2C主模式向IST8310寄存器写入数据
  * @param  addr: IST8310寄存器的写入地址
  *         data: 要写入的数据
  * @retval   
  * @usage  在ist8310_init() 函数中调用
  */
static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data)
{
    /* 首先关闭Slave 1 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, addr);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, data);
    MPU_DELAY(2);
    /* 打开Slave 1，传输一个字节 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    /* 等待更长时间以确保数据从Slave 1传输完成 */
    MPU_DELAY(10);
}

/**
  * @brief  通过MPU6500的I2C主模式从IST8310寄存器读取数据
  * @param  addr: IST8310寄存器的读取地址
  * @retval 
  * @usage  在ist8310_init() 函数中调用
  */
static uint8_t ist_reg_read_by_mpu(uint8_t addr)
{
    uint8_t retval;
    mpu_write_byte(MPU6500_I2C_SLV4_REG, addr);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x80);
    MPU_DELAY(10);
    retval = mpu_read_byte(MPU6500_I2C_SLV4_DI);
    /* 读取后关闭Slave 4 */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    MPU_DELAY(10);
    return retval;
}

/**
  * @brief   初始化MPU6500的I2C Slave 0以进行I2C读取
  * @param    device_address: 从设备地址, Address[6:0]
  * @retval   void
  * @note     
  */
static void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
    /* 
     * 配置IST8310的设备地址
     * 使用Slave1，自动传输单次测量模式
     */
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, device_address);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
    MPU_DELAY(2);

    /* 使用Slave0，自动读取数据 */
    mpu_write_byte(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV0_REG, reg_base_addr);
    MPU_DELAY(2);

    /* 每8个MPU6500内部采样一次I2C主读取 */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x03);
    MPU_DELAY(2);
    /* 启用Slave 0和1的访问延迟 */
    mpu_write_byte(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    MPU_DELAY(2);
    /* 启用Slave 1的自动传输 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    /* 等待6ms（16次内部平均设置的最小等待时间） */
    MPU_DELAY(6); 
    /* 启用Slave 0，读取data_num字节 */
    mpu_write_byte(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
    MPU_DELAY(2);
}

/**
  * @brief  初始化IST8310设备
  * @param  
  * @retval 
  * @usage  在mpu_device_init() 函数中调用
  */
uint8_t ist8310_init()
{
    /* 启用I2C主模式 */
    mpu_write_byte(MPU6500_USER_CTRL, 0x30);
    MPU_DELAY(10);
    /* 启用I2C 400kHz */
    mpu_write_byte(MPU6500_I2C_MST_CTRL, 0x0d); 
    MPU_DELAY(10);

    /* 打开Slave 1用于IST写入，Slave 4用于IST读取 */
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);  
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
    MPU_DELAY(10);

    /* IST8310_R_CONFB 0x01 = 设备复位 */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
    MPU_DELAY(10);
    if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
        return 1;

    /* 软复位 */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01); 
    MPU_DELAY(10);

    /* 配置为准备模式以访问寄存器 */
    ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00); 
    if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
        return 2;
    MPU_DELAY(10);

    /* 正常状态，无中断 */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x00);
    if (ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
        return 3;
    MPU_DELAY(10);
    
    /* 配置低噪声模式，x,y,z轴16次1次平均 */
    ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24); //100100
    if (ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
        return 4;
    MPU_DELAY(10);

    /* 设置/复位脉冲持续时间，正常模式 */
    ist_reg_write_by_mpu(IST8310_PDCNTL, 0xc0);
    if (ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
        return 5;
    MPU_DELAY(10);

    /* 关闭Slave1和Slave4 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    MPU_DELAY(10);

    /* 配置并打开Slave 0 */
    mpu_master_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
    MPU_DELAY(100);
    return 0;
}

/**
  * @brief  获取IST8310的数据
  * @param  buff: 保存IST8310数据的缓冲区
  * @retval 
  * @usage  在mpu_get_data() 函数中调用
  */
void ist8310_get_data(uint8_t* buff)
{
    mpu_read_bytes(MPU6500_EXT_SENS_DATA_00, buff, 6); 
}

/**
  * @brief  获取IMU的数据
  * @param  
  * @retval 
  * @usage  在main() 函数中调用
  */
void mpu_get_data()
{
    mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

    mpu_data.ax   = mpu_buff[0] << 8 | mpu_buff[1];
    mpu_data.ay   = mpu_buff[2] << 8 | mpu_buff[3];
    mpu_data.az   = mpu_buff[4] << 8 | mpu_buff[5];
    mpu_data.temp = mpu_buff[6] << 8 | mpu_buff[7];

    mpu_data.gx = ((mpu_buff[8]  << 8 | mpu_buff[9])  - mpu_data.gx_offset);
    mpu_data.gy = ((mpu_buff[10] << 8 | mpu_buff[11]) - mpu_data.gy_offset);
    mpu_data.gz = ((mpu_buff[12] << 8 | mpu_buff[13]) - mpu_data.gz_offset);

    ist8310_get_data(ist_buff);
    memcpy(&mpu_data.mx, ist_buff, 6);

    memcpy(&imu.ax, &mpu_data.ax, 6 * sizeof(int16_t));
    
    imu.temp = 21 + mpu_data.temp / 333.87f;
    /* 2000dps -> rad/s */
    imu.wx   = mpu_data.gx / 16.384f / 57.3f; 
    imu.wy   = mpu_data.gy / 16.384f / 57.3f; 
    imu.wz   = mpu_data.gz / 16.384f / 57.3f;
}

/**
  * @brief  设置IMU 6500陀螺仪的测量范围
  * @param  fsr: 范围(0,±250dps;1,±500dps;2,±1000dps;3,±2000dps)
  * @retval 
  * @usage  在mpu_device_init() 函数中调用
  */
uint8_t mpu_set_gyro_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_GYRO_CONFIG, fsr << 3);
}

/**
  * @brief  设置IMU 6050/6500加速度计的测量范围
  * @param  fsr: 范围(0,±2g;1,±4g;2,±8g;3,±16g)
  * @retval 
  * @usage  在mpu_device_init() 函数中调用
  */
uint8_t mpu_set_accel_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_ACCEL_CONFIG, fsr << 3); 
}

uint8_t id;

/**
  * @brief  初始化IMU MPU6500和磁力计IST3810
  * @param  
  * @retval 
  * @usage  在main() 函数中调用
  */
uint8_t mpu_device_init(void)
{
    MPU_DELAY(100);

    id                               = mpu_read_byte(MPU6500_WHO_AM_I);
    uint8_t i                        = 0;
    uint8_t MPU6500_Init_Data[10][2] = {{ MPU6500_PWR_MGMT_1, 0x80 },     /* 复位设备 */ 
                                        { MPU6500_PWR_MGMT_1, 0x03 },     /* 时钟源 - Gyro-Z */ 
                                        { MPU6500_PWR_MGMT_2, 0x00 },     /* 启用加速度计和陀螺仪 */ 
                                        { MPU6500_CONFIG, 0x04 },         /* LPF 41Hz */ 
                                        { MPU6500_GYRO_CONFIG, 0x18 },    /* +-2000dps */ 
                                        { MPU6500_ACCEL_CONFIG, 0x10 },   /* +-8G */ 
                                        { MPU6500_ACCEL_CONFIG_2, 0x02 }, /* 启用低通滤波器 */ 
                                        { MPU6500_USER_CTRL, 0x20 },};    /* 启用AUX */ 
    for (i = 0; i < 10; i++)
    {
        mpu_write_byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
        MPU_DELAY(1);
    }

    mpu_set_gyro_fsr(3);         
    mpu_set_accel_fsr(2);

    ist8310_init();
    mpu_offset_call();
    return 0;
}

/**
  * @brief  获取MPU6500的偏移数据
  * @param  
  * @retval 
  * @usage  在main() 函数中调用
  */
void mpu_offset_call(void)
{
    int i;
    for (i=0; i<300;i++)
    {
        mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

        mpu_data.ax_offset += mpu_buff[0] << 8 | mpu_buff[1];
        mpu_data.ay_offset += mpu_buff[2] << 8 | mpu_buff[3];
        mpu_data.az_offset += mpu_buff[4] << 8 | mpu_buff[5];
    
        mpu_data.gx_offset += mpu_buff[8]  << 8 | mpu_buff[9];
        mpu_data.gy_offset += mpu_buff[10] << 8 | mpu_buff[11];
        mpu_data.gz_offset += mpu_buff[12] << 8 | mpu_buff[13];

        MPU_DELAY(5);
    }
    mpu_data.ax_offset=mpu_data.ax_offset / 300;
    mpu_data.ay_offset=mpu_data.ay_offset / 300;
    mpu_data.az_offset=mpu_data.az_offset / 300;
    mpu_data.gx_offset=mpu_data.gx_offset / 300;
    mpu_data.gy_offset=mpu_data.gy_offset / 300;
    mpu_data.gz_offset=mpu_data.gz_offset / 300;
}

/**
  * @brief  初始化四元数
  * @param  
  * @retval 
  * @usage  在main() 函数中调用
  */
void init_quaternion(void)
{
    int16_t hx, hy;//hz;
    
    hx = imu.mx;
    hy = imu.my;
    //hz = imu.mz;
    
    #ifdef BOARD_DOWN
    if (hx < 0 && hy < 0) 
    {
        if (fabs(hx / hy) >= 1)
        {
            q0 = -0.005;
            q1 = -0.199;
            q2 = 0.979;
            q3 = -0.0089;
        }
        else
        {
            q0 = -0.008;
            q1 = -0.555;
            q2 = 0.83;
            q3 = -0.002;
        }
        
    }
    else if (hx < 0 && hy > 0)
    {
        if (fabs(hx / hy)>=1)   
        {
            q0 = 0.005;
            q1 = -0.199;
            q2 = -0.978;
            q3 = 0.012;
        }
        else
        {
            q0 = 0.005;
            q1 = -0.553;
            q2 = -0.83;
            q3 = -0.0023;
        }
        
    }
    else if (hx > 0 && hy > 0)
    {
        if (fabs(hx / hy) >= 1)
        {
            q0 = 0.0012;
            q1 = -0.978;
            q2 = -0.199;
            q3 = -0.005;
        }
        else
        {
            q0 = 0.0023;
            q1 = -0.83;
            q2 = -0.553;
            q3 = 0.0023;
        }
        
    }
    else if (hx > 0 && hy < 0)
    {
        if (fabs(hx / hy) >= 1)
        {
            q0 = 0.0025;
            q1 = 0.978;
            q2 = -0.199;
            q3 = 0.008;            
        }
        else
        {
            q0 = 0.0025;
            q1 = 0.83;
            q2 = -0.56;
            q3 = 0.0045;
        }        
    }
    #else
        if (hx < 0 && hy < 0)
    {
        if (fabs(hx / hy) >= 1)
        {
            q0 = 0.195;
            q1 = -0.015;
            q2 = 0.0043;
            q3 = 0.979;
        }
        else
        {
            q0 = 0.555;
            q1 = -0.015;
            q2 = 0.006;
            q3 = 0.829;
        }
        
    }
    else if (hx < 0 && hy > 0)
    {
        if(fabs(hx / hy) >= 1)
        {
            q0 = -0.193;
            q1 = -0.009;
            q2 = -0.006;
            q3 = 0.979;
        }
        else
        {
            q0 = -0.552;
            q1 = -0.0048;
            q2 = -0.0115;
            q3 = 0.8313;
        }
        
    }
    else if (hx > 0 && hy > 0)
    {
        if(fabs(hx / hy) >= 1)
        {
            q0 = -0.9785;
            q1 = 0.008;
            q2 = -0.02;
            q3 = 0.195;
        }
        else
        {
            q0 = -0.9828;
            q1 = 0.002;
            q2 = -0.0167;
            q3 = 0.5557;
        }
        
    }
    else if (hx > 0 && hy < 0)
    {
        if(fabs(hx / hy) >= 1)
        {
            q0 = -0.979;
            q1 = 0.0116;
            q2 = -0.0167;
            q3 = -0.195;            
        }
        else
        {
            q0 = -0.83;
            q1 = 0.014;
            q2 = -0.012;
            q3 = -0.556;
        }        
    }
    #endif
}

/**
  * @brief  更新IMU的AHRS（姿态航向参考系统）
  * @param  
  * @retval 
  * @usage  在main() 函数中调用
  */
void imu_ahrs_update(void) 
{
    float norm;
    float hx, hy, hz, bx, bz;
    double vx, vy, vz, wx, wy, wz;
    float ex, ey, ez, halfT;
    float tempq0,tempq1,tempq2,tempq3;

    double q0q0 = q0*q0;
    double q0q1 = q0*q1;
    double q0q2 = q0*q2;
    double q0q3 = q0*q3;
    double q1q1 = q1*q1;
    double q1q2 = q1*q2;
    double q1q3 = q1*q3;
    double q2q2 = q2*q2;   
    double q2q3 = q2*q3;
    double q3q3 = q3*q3;   

    gx = imu.wx;
    gy = imu.wy;
    gz = imu.wz;
    ax = imu.ax;
    ay = imu.ay;
    az = imu.az;
    mx = imu.mx;
    my = imu.my;
    mz = imu.mz;

    now_update  = HAL_GetTick(); //ms
    halfT       = ((float)(now_update - last_update) / 2000.0f);
    last_update = now_update;
    
    /* 快速计算1/Sqrt(x) */
    norm = inv_sqrt(ax*ax + ay*ay + az*az);       
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    
    #ifdef IST8310
        norm = inv_sqrt(mx*mx + my*my + mz*mz);          
        mx = mx * norm;
        my = my * norm;
        mz = mz * norm; 
    #else
        mx = 0;
        my = 0;
        mz = 0;        
    #endif
    /* 计算磁力计参考方向 */
    hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
    hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
    hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz; 
    
    /* 估计重力和磁力计的方向（v和w） */
    vx = 2.0f*(q1q3 - q0q2);
    vy = 2.0f*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
    wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
    wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
    
    /* 
     * 误差是参考方向与传感器测量方向的叉积之和
     */
    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

    /* PI控制 */
    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
        exInt = exInt + ex * Ki * halfT;
        eyInt = eyInt + ey * Ki * halfT;    
        ezInt = ezInt + ez * Ki * halfT;
        
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
    }
    
    tempq0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
    tempq1 = q1 + (q0*gx + q2*gz - q3*gy) * halfT;
    tempq2 = q2 + (q0*gy - q1*gz + q3*gx) * halfT;
    tempq3 = q3 + (q0*gz + q1*gy - q2*gx) * halfT;  

    /* 归一化四元数 */
    norm = inv_sqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;
		
		imu.vx = vx;
		imu.vy = vy;
		imu.vz = vz;
}

/**
  * @brief  更新IMU的姿态
  * @param  
  * @retval 
  * @usage  在main() 函数中调用
  */
void imu_attitude_update(void)
{
    /* 偏航角    -pi----pi */
    imu.yaw = -atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1); 
    /* 俯仰角  -pi/2----pi/2 */
    imu.pit = -asin(-2*q1*q3 + 2*q0*q2);   
    /* 横滚角   -pi----pi  */    
    imu.rol =  atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1);
}
