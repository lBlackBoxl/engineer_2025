#include "can_communicate.h"
#include "chassis_task.h"
#include "init.h"
#include "main.h"
#include "boards_communicate.h"
#include "arm_control_task.h"
#include "can.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define get_motor_measure(ptr, data)/*读取电机数据 */															\
{																																									\
	(ptr)->last_ecd = (ptr)->ecd;																										\
	(ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);														\
	(ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);											\
	(ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);									\
	(ptr)->temperate = (data)[6];																										\
}

#define get_DM_motor_measure(ptr, data)                                                                 \
{                                                                                                       \
	uint16_t int_position,int_speed,int_T;                                                                \
                                                                                                        \
	int_position = (uint16_t)((data[1]<<8)|data[2]);                                                      \
	int_speed = (uint16_t)((data[3]<<4)|((data[4]&0xF0)>>4));/*达妙反馈帧详情看说明书*/                   \
  int_T = (uint16_t)(((data[4]&0x0F)<<8)|data[5]);                                                      \
	                                                                                                      \
	(ptr)->motor_err = data[0] & 0xF0;                                                                    \
	(ptr)->motor_tx_id = data[0] & 0x0F;                                                                  \
	(ptr)->motor_enabled = data[0]>>4;                                                                    \
	(ptr)->motor_position = ((fp32)int_position)*3.141593f*2.0f/((fp32)((1<<16)-1))-3.141593f;            \
	(ptr)->motor_speed = ((fp32)int_speed)*20.0f*2.0f/((fp32)((1<<12)-1))-20.0f;                          \
	(ptr)->motor_T = ((fp32)int_T)*10.0f*2.0f/((fp32)((1<<12)-1))-10.0f;                                  \
}

#define get_board_communicate_data_0(ptr, data)                     			\
{                                                                   			\
    (ptr)->target_position[0] = (uint16_t)((data)[0] << 8 | (data)[1]);  	\
    (ptr)->target_position[1] = (uint16_t)((data)[2] << 8 | (data)[3]);   \
    (ptr)->target_position[2] = (uint16_t)((data)[4] << 8 | (data)[5]);   \
    (ptr)->target_position[3] = (uint16_t)((data)[6] << 8 | (data)[7]);  	\
}

#define get_board_communicate_data_1(ptr, data)                     			\
{                                                                   			\
    (ptr)->target_position[4] = (uint16_t)((data)[0] << 8 | (data)[1]);  	\
    (ptr)->target_position[5] = (uint16_t)((data)[2] << 8 | (data)[3]);   \
		(ptr)->error_info = (uint16_t)((data)[4] << 8 | (data)[5]);						\
}		

motor_measure_t CAN1_rx_buffer[5];//0~3-底盘4个3508 4履带3508
motor_DM_motor_t DM_motor_4310[2];//1台阶，2YAW

arm_communicate arm_message;
arm_communicate arm_message_temp;

static CAN_TxHeaderTypeDef  chassis_can1_tx_message;
static CAN_TxHeaderTypeDef  chassis_can2_tx_message;
static uint8_t              chassis_can1_send_data[8];
static uint8_t              chassis_can2_send_data[8];

int float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
  float offset = x_min;
  return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		
	if(hcan == &CHASSIS_CAN)
	{
    switch (rx_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
				case CAN_2006_clamp_ID:
        {
            static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - CAN_3508_M1_ID;
            get_motor_measure(&CAN1_rx_buffer[i], rx_data);
								if(i == 4)
								{
									if(CAN1_rx_buffer[4].ecd - CAN1_rx_buffer[4].last_ecd > HALF_ECD_RANGE)
									{
										chassis.motor_clamp.round_cnt--;
									}
									else if(CAN1_rx_buffer[4].ecd - CAN1_rx_buffer[4].last_ecd < -HALF_ECD_RANGE)
									{
										chassis.motor_clamp.round_cnt++;
									}
								}
            break;
        }
				default:
				{
					  break;
				}
			}
		}
	 else if(hcan == &BROADS_CAN)
	 {
			 switch (rx_header.StdId)
			 {
					case MST_1_ID:
					{
							get_DM_motor_measure(&DM_motor_4310[0], rx_data);
							break;
					}
					case MST_YAW_ID:
					{
							get_DM_motor_measure(&DM_motor_4310[1], rx_data);
							break;
					}
					case CAN_COMMUNICATE_RX_ID_0:
					{
							get_board_communicate_data_0(&arm_message_temp, rx_data);
							arm_message.target_position[0] = arm_control.motor_YAW_data.DM_motor_measure->motor_position;
							arm_message.target_position[1] = uint_to_float(arm_message_temp.target_position[1], -6.2831852f, 6.2831852f, 16);
							arm_message.target_position[2] = uint_to_float(arm_message_temp.target_position[2], -6.2831852f, 6.2831852f, 16);
							arm_message.target_position[3] = uint_to_float(arm_message_temp.target_position[3], -6.2831852f, 6.2831852f, 16);
							break;
					}
					case CAN_COMMUNICATE_RX_ID_1:
					{
							get_board_communicate_data_1(&arm_message_temp, rx_data);
							arm_message.target_position[4] = uint_to_float(arm_message_temp.target_position[4], -6.2831852f, 6.2831852f, 16);
							arm_message.target_position[5] = uint_to_float(arm_message_temp.target_position[5], -6.2831852f, 6.2831852f, 16);
							chassis.error_info = arm_message.error_info;
							break;
					}
					default:
					{
							break;
					}
				}
    }
}

void CAN_cmd_4310_enable(uint8_t DM_ID, CAN_HandleTypeDef hcan)
{
	uint32_t send_mail_box;
  chassis_can2_tx_message.StdId = DM_ID;
  chassis_can2_tx_message.IDE = CAN_ID_STD;
  chassis_can2_tx_message.RTR = CAN_RTR_DATA;
	chassis_can2_tx_message.DLC = 0x08;
	chassis_can2_send_data[0] = 0xFF;
	chassis_can2_send_data[1] = 0xFF;
	chassis_can2_send_data[2] = 0xFF;
	chassis_can2_send_data[3] = 0xFF;
	chassis_can2_send_data[4] = 0xFF;
	chassis_can2_send_data[5] = 0xFF;
	chassis_can2_send_data[6] = 0xFF;
	chassis_can2_send_data[7] = 0xFC;
	HAL_CAN_AddTxMessage(&hcan, &chassis_can2_tx_message, chassis_can2_send_data, &send_mail_box);
}


void CAN_cmd_4310_disable(uint8_t DM_ID, CAN_HandleTypeDef hcan)
{
	uint32_t send_mail_box;
  chassis_can2_tx_message.StdId = DM_ID;
  chassis_can2_tx_message.IDE = CAN_ID_STD;
  chassis_can2_tx_message.RTR = CAN_RTR_DATA;
	chassis_can2_tx_message.DLC = 0x08;
	chassis_can2_send_data[0] = 0xFF;
	chassis_can2_send_data[1] = 0xFF;
	chassis_can2_send_data[2] = 0xFF;
	chassis_can2_send_data[3] = 0xFF;
	chassis_can2_send_data[4] = 0xFF;
	chassis_can2_send_data[5] = 0xFF;
	chassis_can2_send_data[6] = 0xFF;
	chassis_can2_send_data[7] = 0xFD;
	HAL_CAN_AddTxMessage(&hcan, &chassis_can2_tx_message, chassis_can2_send_data, &send_mail_box);
}


void CAN_cmd_4310_PV(fp32 DM_motor_position, fp32 DM_motor_speed, uint8_t DM_ID, CAN_HandleTypeDef hcan)
{
	uint8_t *pbuf,*vbuf;
	pbuf = (uint8_t*) &DM_motor_position;
	vbuf = (uint8_t*) &DM_motor_speed;
	uint32_t send_mail_box;
  chassis_can2_tx_message.StdId = 0x100 + DM_ID;
  chassis_can2_tx_message.IDE = CAN_ID_STD;
  chassis_can2_tx_message.RTR = CAN_RTR_DATA;
	chassis_can2_tx_message.DLC = 0x08;
	chassis_can2_send_data[0] = *pbuf;
	chassis_can2_send_data[1] = *(pbuf+1);
	chassis_can2_send_data[2] = *(pbuf+2);
	chassis_can2_send_data[3] = *(pbuf+3);
	chassis_can2_send_data[4] = *vbuf;
	chassis_can2_send_data[5] = *(vbuf+1);
	chassis_can2_send_data[6] = *(vbuf+2);
	chassis_can2_send_data[7] = *(vbuf+3);
	HAL_CAN_AddTxMessage(&hcan, &chassis_can2_tx_message, chassis_can2_send_data, &send_mail_box);
}


void CAN_cmd_4310_mit(fp32 DM_motor_position,fp32 DM_motor_speed,fp32 Kp,fp32 Kd,fp32 torq,uint8_t DM_ID, CAN_HandleTypeDef hcan)
{
	  uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	  pos_tmp = float_to_uint(DM_motor_position, -3.141593f, 3.141593f, 16);
    vel_tmp = float_to_uint(DM_motor_speed, -20.0f, 20.0f, 12);
    kp_tmp  = float_to_uint(Kp, 0, 500.0f, 12);
    kd_tmp  = float_to_uint(Kd, 0, 5.0f, 12);
    tor_tmp = float_to_uint(torq, -10.0f, 10.0f, 12);
    uint32_t send_mail_box;
    chassis_can2_tx_message.StdId = DM_ID;
    chassis_can2_tx_message.IDE = CAN_ID_STD;
    chassis_can2_tx_message.RTR = CAN_RTR_DATA;
    chassis_can2_tx_message.DLC = 0x08;
    chassis_can2_send_data[0] = (pos_tmp >> 8);
    chassis_can2_send_data[1] =  pos_tmp;
    chassis_can2_send_data[2] = (vel_tmp >> 4);
    chassis_can2_send_data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    chassis_can2_send_data[4] = kp_tmp;
    chassis_can2_send_data[5] = (kd_tmp >> 4);
    chassis_can2_send_data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    chassis_can2_send_data[7] = tor_tmp;
    HAL_CAN_AddTxMessage(&hcan, &chassis_can2_tx_message, chassis_can2_send_data, &send_mail_box);
}	

void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_can1_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_can1_tx_message.IDE = CAN_ID_STD;
    chassis_can1_tx_message.RTR = CAN_RTR_DATA;
    chassis_can1_tx_message.DLC = 0x08;
    chassis_can1_send_data[0] = motor1 >> 8;
    chassis_can1_send_data[1] = motor1;
    chassis_can1_send_data[2] = motor2 >> 8;
    chassis_can1_send_data[3] = motor2;
    chassis_can1_send_data[4] = motor3 >> 8;
    chassis_can1_send_data[5] = motor3;
    chassis_can1_send_data[6] = motor4 >> 8;
    chassis_can1_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &chassis_can1_tx_message, chassis_can1_send_data, &send_mail_box);
}

void CAN_cmd_chassis_clamp(int16_t motor1)
{
	  uint32_t send_mail_box;
    chassis_can2_tx_message.StdId = 0x1FF;
    chassis_can2_tx_message.IDE = CAN_ID_STD;
    chassis_can2_tx_message.RTR = CAN_RTR_DATA;
    chassis_can2_tx_message.DLC = 0x08;
    chassis_can2_send_data[0] = motor1 >> 8;
    chassis_can2_send_data[1] = motor1;
    chassis_can2_send_data[2] = 0;
    chassis_can2_send_data[3] = 0;
    chassis_can2_send_data[4] = 0;
    chassis_can2_send_data[5] = 0;
    chassis_can2_send_data[6] = 0;
    chassis_can2_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &chassis_can2_tx_message, chassis_can2_send_data, &send_mail_box);
}

const motor_measure_t *get_chassis_motor_point(uint8_t i)
{
  return &CAN1_rx_buffer[i];
}

motor_DM_motor_t *return_4310_measure(uint8_t i)
{
	return &DM_motor_4310[i];
}
