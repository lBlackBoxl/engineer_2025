/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef __CAN_COMMUNICATE_H
#define __CAN_COMMUNICATE_H

#include "struct_typedef.h"
#include "can.h"
#include "arm_math.h"

#define CAN_YAW_TYPE &hcan2
#define CAN_SHOOT_TYPE &hcan1
#define CAN_CHASSIC_COM_TYPE &hcan2
/* CAN send and receive ID */
typedef enum
{
    CAN_SHOOT_ALL_ID    = 0X1FF,// 射击电机标识符
    CAN_FRIC1_M1_ID 		= 0x205,//摩擦轮1(左)
    CAN_FRIC2_M2_ID 		= 0x206,//摩擦轮2(右)
    CAN_TRIGGER_M3_ID	  = 0x207,//发弹电机

    CAN_YAW_ALL_ID			= 0x2FF,//6020 Y轴电机标识符
    CAN_YAW_M5_ID 		  = 0x209,//Y轴 0X204+ID
    CAN_DEMO_ALL_ID     = 0X200,
    CAN_DEMO_ID 		    = 0x204,
		CAN_REFEREE_ID 			= 0x230,
		CAN_COMMUNICATE_ID 		= 0x220,
	  CAN_SPEED_ID  = 0X240,
} can_msg_id_e;
union fire_speed_u
{
    float fire_speed_value;
    uint8_t fire_speed_rx[4];
};

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

extern uint8_t color_flag;
extern union fire_speed_u fire_speed;
extern void CAN_gimbal_yaw_motor_can2(int16_t current);
extern void CAN_gimbal_pitch_motor_can2(int16_t current);
extern void CAN_gimbal_shoot_motor_can1(int16_t Fric1_Motor_Data, int16_t Fric2_Motor_Data,int16_t Trigger_Motor_Data);

extern void CAN_cmd_communication(void);
void USART1_RX_IRQHandler(void);
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);


/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);


/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
extern const motor_measure_t *get_fric2_motor_measure_point(void);
extern const motor_measure_t *get_fric1_motor_measure_point(void);
extern const motor_measure_t *get_trigger_motor_measure_point(void);

#endif
