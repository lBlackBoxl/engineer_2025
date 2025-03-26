#ifndef CAN_COMMUNICATE_H
#define CAN_COMMUNICATE_H

#include "struct_typedef.h"
#include "can.h"

typedef enum
{
	DM_M2_TX_ID = 0x01,
	MST_2_ID = 0x00,
	DM_M3_TX_ID = 0x03,
	MST_3_ID = 0x02,
	DM_M4_TX_ID = 0x05,
	MST_4_ID = 0x04,
	CAN_ARM_ALL_ID = 0x200,
	CAN_2006_M1_ID = 0x201,
	CAN_2006_M2_ID = 0x202,
} can1_msg_id_e;

typedef struct
{
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
	int16_t last_ecd;
} DJ_measure_t;

typedef struct
{
	uint8_t motor_err;
	uint8_t motor_tx_id;
	uint8_t motor_enabled;
	fp32 motor_position;
	fp32 motor_speed;
	fp32 motor_T;
} DM_measure_t;

typedef enum
{
	CAN_COMMUNICATE_RX_ID_0 = 0x220,
	CAN_COMMUNICATE_RX_ID_1 = 0x221,
	CAN_COMMUNICATE_TX_ID_0 = 0x230,
	CAN_COMMUNICATE_TX_ID_1 = 0x231,
}can_board_communicate;

typedef struct
{
	uint16_t target_position[6];
	uint16_t mode;
	uint16_t suker_mode;
}board_communicate;

extern void CAN_cmd_2006(int16_t motor_2006_current_1, int16_t motor_2006_current_2);
extern void CAN_cmd_4310_enable(uint8_t DM_ID, CAN_HandleTypeDef hcan);
extern void CAN_cmd_4310_disable(uint8_t DM_ID, CAN_HandleTypeDef hcan);
extern void CAN_cmd_4310_mit(fp32 DM_motor_position,fp32 DM_motor_speed,fp32 Kp,fp32 Kd,fp32 torq,uint8_t DM_ID, CAN_HandleTypeDef hcan);
extern void CAN_cmd_4310_PV(fp32 DM_motor_position, fp32 DM_motor_speed, uint8_t DM_ID, CAN_HandleTypeDef hcan);
extern void CAN_board_communicate_can2_0(fp32 arm_position_message[6]);
extern void CAN_board_communicate_can2_1(fp32 arm_position_message[6]);
extern const DJ_measure_t *get_chassis_motor_point(uint8_t i);
extern DM_measure_t *return_4310_measure(uint8_t i);

extern board_communicate board_message_temp;
extern board_communicate board_message;
extern uint32_t Motor_3508_Timeout_Count;
extern uint8_t  Motor_3508_Timeout_Flag;

#endif