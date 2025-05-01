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

#include "can_communicate.h"
//#include "detect_task.h"
#include "shoot.h"
#include "gimbal_task.h"
#include "bsp_usart.h"
#include "usart.h"
#include "CyberGear.h"
#include "remote_control.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
extern shoot_control_t shoot_control;//射击数据
uint8_t color_flag;
union fire_speed_u fire_speed;
static int16_t can_motor_senddata[4];
static uint32_t send_mail_box;
static CAN_TxHeaderTypeDef  can_tx_message;
static uint8_t  can_send_data[8];
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

static motor_measure_t motor_gimbal[7]; //电机回传参数

static void get_referee_measure(uint8_t rx_data[8]);
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//    CAN_RxHeaderTypeDef rx_header;
//    uint8_t rx_data[8];
//    static uint8_t i = 0;
//    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
//    if (hcan == CAN_SHOOT_TYPE){
//        switch (rx_header.StdId)
//        {
//        case CAN_FRIC1_M1_ID:
//        case CAN_FRIC2_M2_ID:
//        case CAN_TRIGGER_M3_ID:
//        {
//					  i = rx_header.StdId - CAN_FRIC1_M1_ID;
//            get_motor_measure(&motor_gimbal[i], rx_data);
//            if(rx_header.StdId == CAN_TRIGGER_M3_ID)
//            {
//                shoot_trigger_angle_update();
//            }

//            break;
//        }
//        default:
//        {
//            break;
//        }
//        }
//    }
//    if (hcan == CAN_YAW_TYPE) {
//        switch (rx_header.StdId)
//        {
//        case CAN_YAW_M5_ID:
//        {
//            get_motor_measure(&motor_gimbal[4], rx_data);
//            break;
//        }
//        }
//    }
//    if (hcan == CAN_CHASSIC_COM_TYPE) {
//        switch (rx_header.StdId)
//        {
//        case CAN_REFEREE_ID:
//        {
//            get_referee_measure(rx_data);
//            break;
//        }
//				case CAN_SPEED_ID:{
//					memcpy(fire_speed.fire_speed_rx,rx_data,sizeof(rx_data));
//				}
//        }
//    }

//}
void CAN_Motors_SendData(CAN_HandleTypeDef *hcan,uint32_t can_motor_id,int16_t *motor_senddata)
{
    can_tx_message.StdId = can_motor_id;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = (motor_senddata[0] >> 8);
    can_send_data[1] = motor_senddata[0];
    can_send_data[2] = (motor_senddata[1] >> 8);
    can_send_data[3] = motor_senddata[1];
    can_send_data[4] = (motor_senddata[2] >> 8);
    can_send_data[5] = motor_senddata[2];
    can_send_data[6] = (motor_senddata[3] >> 8);
    can_send_data[7] = motor_senddata[3];
    HAL_CAN_AddTxMessage(hcan, &can_tx_message, can_send_data, &send_mail_box);
}
//发送YAW电机电流
void CAN_gimbal_yaw_motor_can2(int16_t current) {

    can_motor_senddata[0] = current;
    can_motor_senddata[1] = 0;
    can_motor_senddata[2] = 0;
    can_motor_senddata[3] = 0;
    CAN_Motors_SendData(CAN_YAW_TYPE,CAN_YAW_ALL_ID,can_motor_senddata);
}
//发送Pitch电机电流
//void CAN_gimbal_pitch_motor_can1(int16_t current) {

//    can_motor_senddata[0] = current;
//    can_motor_senddata[1] = 0;
//    can_motor_senddata[2] = 0;
//    can_motor_senddata[3] = 0;
//    CAN_Motors_SendData(&hcan1,CAN_PITCH_ALL_ID,can_motor_senddata);
//}
//发送摩擦轮，拨弹电机的电流
void CAN_gimbal_shoot_motor_can1(int16_t Fric1_Motor_Data, int16_t Fric2_Motor_Data,int16_t Trigger_Motor_Data) {

    can_motor_senddata[0] = Fric1_Motor_Data;
    can_motor_senddata[1] = Fric2_Motor_Data;
    can_motor_senddata[2] = Trigger_Motor_Data;
    can_motor_senddata[3] = 0;
    CAN_Motors_SendData(CAN_SHOOT_TYPE,CAN_SHOOT_ALL_ID,can_motor_senddata);
}
//云台向底盘发送标志位
void CAN_cmd_communication(void)
{
    can_tx_message.StdId = CAN_COMMUNICATE_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = block_flag;		      //堵转标志位
    can_send_data[1] = shoot_fire_state;		// 摩擦轮开启
    can_send_data[2] = remote_data.key_g.key_switch;		// 单连发模式切换
    can_send_data[3] = remote_data.key_b.key_switch;	  // 自瞄模式（普通-打符）
    can_send_data[4] = remote_data.key_v.key_switch;		//高低速
    can_send_data[5] = 0;
    can_send_data[6] = 0;
    can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, can_send_data, &send_mail_box);
}

/**
  * @brief          返回底盘电机数据指针
  * @param[in]      i: 电机编号,范围[0,6]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_gimbal[(i & 0x07)];
}
/**
  * @brief          返回摩擦轮Fric1电机数据指针
  * @param[in]
  * @retval         电机数据指针
  */
const motor_measure_t *get_fric1_motor_measure_point(void)
{
    return &motor_gimbal[0];
}
/**
  * @brief          返回摩擦轮Fric2电机数据指针
  * @param[in]
  * @retval         电机数据指针
  */
const motor_measure_t *get_fric2_motor_measure_point(void)
{
    return &motor_gimbal[1];
}
/**
  * @brief          返回摩擦轮发射电机数据指针
  * @param[in]
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_gimbal[2];
}
/**
  * @brief          返回PITCH轴电机数据指针
  * @param[in]
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_gimbal[3];
}
/**
  * @brief          返回YAW轴电机数据指针
  * @param[in]
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_gimbal[4];
}

void get_referee_measure(uint8_t rx_data[8])
{
    shoot_control.shoot_referee_data.heat_limit= rx_data[0] << 8 | rx_data[1];
    shoot_control.shoot_referee_data.heat = rx_data[2] << 8 | rx_data[3];
    shoot_control.shoot_referee_data.cooling_value =rx_data[4]<<8 | rx_data[5];
    color_flag = rx_data[7];
}


