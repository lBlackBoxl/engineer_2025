/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       IST8310.c/h
  * @brief      IST8310磁力计驱动函数，包括初始化函数，处理数据函数，通信读取函数
  *             本工程是将MPU6500 IIC_SLV0设置为自动读取IST8310数据，读取
  *             MPU_EXT_SENS_DATA_00保存了IST8310的Status，通过判断标志位，来更新
  *             数据。
  * @note       IST8310只支持IIC读取
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#ifndef IST8310DRIVER_H
#define IST8310DRIVER_H

#include "struct_typedef.h"

#define IST8310_DATA_READY_BIT 2
//T磁感应强度
#define MAG_SEN 0.3f //转换成 uT

#define IST8310_IIC_ADDRESS (0x0E << 1)  //IST8310的IIC地址

#define IST8310_WHO_AM_I 0x00       //ist8310 who am I ID寄存器
#define IST8310_WHO_AM_I_VALUE 0x10 //设备 ID

#define IST8310_NO_ERROR 0x00 //没有错误

#define IST8310_NO_SENSOR 0x40//没有传感器

typedef struct ist8310_real_data_t
{
    uint8_t status;   //状态
    fp32 mag[3];    //值
} ist8310_real_data_t;
extern uint8_t ist8310_init(void);

#endif
