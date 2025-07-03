#ifndef _SMS_STS_H
#define _SMS_STS_H

#include "main.h"

// 内存表定义
//-------EPROM(只读)--------
#define STS_Firmware_master_version 0x0
#define STS_DATA_LENGHT_0 1

#define STS_Firmware_sub_version 0x1
#define STS_DATA_LENGHT_1 1

#define STS_End 0x2
#define STS_DATA_LENGHT_2 1

#define STS_servo_master_version 0x3
#define STS_DATA_LENGHT_3 1

#define STS_servo_sub_version 0x4
#define STS_DATA_LENGHT_4 1

//-------EPROM(读写)--------
#define STS_ID_ADDR 0x5
#define STS_DATA_LENGHT_5 1

#define STS_BAUD_RATE 0x6
#define STS_DATA_LENGHT_6 1
// 波特率定义
#define BAUD_RATE_1000000 0
#define BAUD_RATE_500000 1
#define BAUD_RATE_250000 2
#define BAUD_RATE_128000 3
#define BAUD_RATE_115200 4
#define BAUD_RATE_76800 5
#define BAUD_RATE_57600 6
#define BAUD_RATE_38400 7

#define STS_RESPONSE_STATUS 0x8 // 响应状态定义 0：只有读指令和ping指令返回应答包 1.对所有指令返回应答包
#define STS_DATA_LENGHT_8 1

#define STS_MIN_ANGLE_LIMIT 0x9
#define STS_DATA_LENGHT_9 2

#define STS_MAX_ANGLE_LIMIT 0xB
#define STS_DATA_LENGHT_B 2

#define STS_MAX_TEMPERATURE_LIMIT 0xD
#define STS_DATA_LENGHT_D 1

#define STS_MAX_VOLTAGE_LIMIT 0xE
#define STS_DATA_LENGHT_E 1

#define STS_MIN_VOLTAGE_LIMIT 0xF
#define STS_DATA_LENGHT_F 1

#define STS_MAX_TORQUE_LIMIT 0x10
#define STS_DATA_LENGHT_10 2

#define STS_UNLOAD_CONDITION 0x13  //对应位设置1为开启相应保护，对应位置0为关闭相应保护
#define STS_DATA_LENGHT_13 1

#define STS_ALARM_LED 0x14 //对应位设置1为开启闪灯报警，对应位置0为关闭闪灯报警
#define STS_DATA_LENGHT_14 1

#define STS_PID_KP 0x15  //0~254
#define STS_DATA_LENGHT_15 1

#define STS_PID_KD 0x16  //0~254
#define STS_DATA_LENGHT_16 1

#define STS_PID_KI 0x17  //0~254
#define STS_DATA_LENGHT_17 1

#define STS_MIN_TORQUE_ENABLE 0x18
#define STS_DATA_LENGHT_18 1

#define STS_INTEGRAL_LIMIT 0x19 //最大积分值=积分限制值*4，0表示关闭积分限制功能，位置模式与步进模式生效
#define STS_DATA_LENGHT_19 1

#define STS_CLOCKWISE_DEAD_ZONE 0x1A //0~32 单位为最小分辨角度（0.0879°）
#define STS_DATA_LENGHT_1A 1

#define STS_ANTICLOCKWISE_DEAD_ZONE 0x1B //0~32 单位为最小分辨角度（0.0879°）
#define STS_DATA_LENGHT_1B 1

#define STS_PROTECTIVE_CURRENT 0x1C //0~511 单位为6.5mA
#define STS_DATA_LENGHT_1C 2

#define STS_POSITION_CALIBRATION 0x1F //-2047~2047 单位为最小分辨角度（0.0879°）
#define STS_DATA_LENGHT_1F 2

#define STS_RUNNING_MODE 0x21 //0：位置模式 1：速度模式 2：PWM开环速度模式 3：步进模式
#define STS_DATA_LENGHT_21 1
// 运行模式定义
#define STS_POSITION_MODE 0
#define STS_SPEED_MODE 1
#define STS_PWM_MODE 2
#define STS_STEP_MODE 3

#define STS_PROTECTIVE_TORQUE 0x22 //0~100 单位为最大扭矩的百分比
#define STS_DATA_LENGHT_22 1

#define STS_PROTECTIVE_TIME 0x23 //0~254 单位为0.01s
#define STS_DATA_LENGHT_23 1

#define STS_OVERLOAD_TORQUE 0x24 //0~100 单位为最大扭矩的百分比
#define STS_DATA_LENGHT_24 1

#define STS_PID_KP_SPEED 0x25 //0~254
#define STS_DATA_LENGHT_25 1

#define STS_OVERLOAD_CURRENT_TIME 0x26 //0~254 单位为0.01s
#define STS_DATA_LENGHT_26 1

#define STS_PID_KI_SPEED 0x27 //0~254
#define STS_DATA_LENGHT_27 1

//-------SRAM(读写)--------
#define STS_TORQUE_ENABLE 0x28 // 0：关闭扭力输出 1：开启扭力输出 128:任意当前位置矫正为2048
#define STS_DATA_LENGHT_28 1

#define STS_ACC_SPEED 0x29 //0~254 单位为100步/秒^2
#define STS_DATA_LENGHT_29 1

#define STS_TORGET_POSITION 0x2A //-32766 ~ 32766 单位为最小分辨角度（0.0879°）
#define STS_DATA_LENGHT_2A 2

#define STS_TORGET_RUNNING_TIME 0x2C //0~1000 单位为0.1%
#define STS_DATA_LENGHT_2C 2

#define STS_TORGET_SPEED 0x2E //-32766 ~ 32766 单位为步/秒
#define STS_DATA_LENGHT_2E 2

#define STS_TORQUE_LIMIT 0x30 //0~100 单位为最大扭矩的百分比
#define STS_DATA_LENGHT_30 2

#define STS_LOCK 0x37 //0:关闭写入锁，写入数据掉电保存 1：开启写入锁，写入数据掉电不保存
#define STS_DATA_LENGHT_37 1

//-------SRAM(只读)--------
#define STS_PRESENT_POSITION 0x38 //单位为步
#define STS_DATA_LENGHT_38 2

#define STS_PRESENT_SPEED 0x3A //单位为步/秒
#define STS_DATA_LENGHT_3A 2

#define STS_PRESENT_LOAD 0x3C //单位为0.1%,表示电压占空比
#define STS_DATA_LENGHT_3C 2

#define STS_PRESENT_VOLTAGE 0x3E //单位为0.1V
#define STS_DATA_LENGHT_3E 1

#define STS_PRESENT_TEMPERATURE 0x3F //单位为摄氏度
#define STS_DATA_LENGHT_3F 1

#define STS_ERROR 0x41
#define STS_DATA_LENGHT_41 1

#define STS_PRESENT_MODE 0x42 //舵机运动时为1，停止时为0
#define STS_DATA_LENGHT_42 1

#define STS_PRESENT_CURRENT 0x45 //单位为6.5mA
#define STS_DATA_LENGHT_45 2

//指令类型定义
#define STS_PING 0x01
#define STS_READ 0x02
#define STS_WRITE 0x03
#define STS_REG_WRITE 0x04
#define STS_ACTION 0x05
#define STS_SYCNREAD 0x82 //用于同时查询多个舵机
#define STS_SYCNWRITE 0x83 //用于同时控制多个舵机
#define STS_RECOVERY 0x06 //恢复舵机出厂设置
#define STS_RESET 0x0A //重置舵机圈数

extern uint8_t STS_CMD_PING(UART_HandleTypeDef *huart, uint8_t STS_ID);
extern uint16_t STS_CMD_READ(UART_HandleTypeDef *huart, uint8_t STS_ID, uint8_t MemAddr, uint8_t rLen);
extern void STS_CMD_WRITE(UART_HandleTypeDef *huart, uint8_t STS_ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);
extern void STS_CMD_ASYNC_WRITE(UART_HandleTypeDef *huart, uint8_t STS_ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);
extern void STS_CMD_ASYNC_ACTION(UART_HandleTypeDef *huart, uint8_t STS_ID);
extern void STS_CMD_SYNC_WRITE(UART_HandleTypeDef *huart, uint8_t STS_ID[], uint8_t IDN, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);
extern uint8_t STS_CMD_SYNC_READ(UART_HandleTypeDef *huart, uint8_t STS_ID[], uint8_t IDN, uint8_t MemAddr, uint8_t *rData[IDN], uint8_t rLen);
extern uint8_t STS_CMD_RECOVERY(UART_HandleTypeDef *huart, uint8_t STS_ID);
extern uint8_t STS_CMD_RESET(UART_HandleTypeDef *huart, uint8_t STS_ID);

#endif