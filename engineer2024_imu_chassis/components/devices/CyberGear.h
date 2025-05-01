/**
  ******************************************************************************
  * @file	 CyberGear.h
  * @author
  * @version V1.0.0
  * @date    2024/3/14
  * @brief
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#ifndef __CYBERGEAR_H
#define __CYBERGEAR_H

#include "struct_typedef.h"
#include "arm_math.h"
#include "can.h"
//对外接口
#define CYBERGEAT_CAN &hcan1
#define CyberGear_CAN_ID    0x7F
#define CyberGear_CAN_MASTER_ID  0x00


#define iq_index 0x7006
#define spd_index 0x700A
#define loc_index 0x7016
#define angle_param 0.0003835069007
#define spd_param 0.0009155552843
#define torque_param 0.0003662221137
#define temp_param 0.1

#define cybergear_RX_BUF_NUM 24u
#define cybergear_RX_BUF_LENGTH 12u


typedef struct
{
    uint8_t mode:2;
    int16_t angle_i;
    int16_t gyro_i;
    int16_t torque_i;
    int16_t temperature_i;
    float32_t angle;
    float32_t gyro;
    float32_t torque;
    float32_t temp;
} cybergear_measure_t;
typedef struct
{
    uint16_t index;
    uint8_t param_one ;
} cybergear_measure_single_t;

//小米电机
struct Tx_exCanIdInfo {
    uint32_t id:8;
    uint32_t data:16;
    uint32_t mode:5;
    uint32_t res:3;
};

typedef enum
{
    move_mode = 0,
    position_mode = 1,
    speed_mode = 2,
    current_mode = 3,
} cybergear_mode;

union Tx_exId_uint32
{
    struct Tx_exCanIdInfo exid;
    uint32_t aa;
};

struct Rx_exCanIdInfo {
    uint32_t id:8;//主机idbit0~7
    uint32_t motor_CanId:8;//电机IDbit8~15
    uint32_t undervoltage_err:1;//欠压故障bit16
    uint32_t overcurrent_err:1;
    uint32_t hightemperature_err:1;
    uint32_t magnetic_coding_err:1;
    uint32_t hall_coding_err:1;
    uint32_t calibration_err:1;
    uint32_t mode:2;//bit22~23模式状态
    uint32_t communication_mode:5;//bit28~bit24通信方式标识
    uint32_t res:3;//预留3位
};

union Rx_exId_uint32
{
    struct Rx_exCanIdInfo exid;
    uint32_t aa;
};
extern cybergear_measure_t cybergear_measure;
extern cybergear_measure_single_t cybergear_measure_single;
extern struct Rx_exCanIdInfo rxCanIdEx;
extern union Rx_exId_uint32 trans_rx;
//CAN
extern void Can_cybergear_runmode_set(uint8_t id, uint16_t master_id, uint8_t runmode);
extern void Can_cybergear_enable(uint8_t id, uint16_t master_id);
extern void Can_cybergear_param_write(uint8_t id, uint16_t master_id, uint16_t index, float32_t param_ref);
extern void Can_cybergear_stop(uint8_t id,uint16_t master_id);
extern void Can_cybergear_param_read(uint8_t id, uint16_t master_id, uint16_t index);
extern void get_cybergear_motor_measure(struct Rx_exCanIdInfo exid, uint8_t rx_data[8]);
extern void get_cybergear_motor_single_para(struct Rx_exCanIdInfo exid, uint8_t rx_data[8]);
//USART
extern void USART_cybergear_init (void);
extern void USART_cybergear_enable(uint8_t id, uint16_t master_id);
extern void USART_cybergear_stop(uint8_t id,uint16_t master_id);
extern void USART_cybergear_param_write(uint8_t id, uint16_t master_id, uint16_t index, float32_t param_ref);
extern void USART_cybergear_runmode_set(uint8_t id, uint16_t master_id, uint8_t runmode);
extern void USART_cybergear_param_read(uint8_t id, uint16_t master_id, uint16_t index);
extern void USART_cybergear_current_set(uint8_t id, uint16_t master_id, float32_t current_num);
#endif
