#ifndef CAN_COMMUNICATE_H
#define CAN_COMMUNICATE_H

#include "main.h"
#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define BROADS_CAN hcan2

typedef struct
{
	fp32 target_position[6];
}arm_communicate;

typedef enum
{
	CAN_CHASSIS_ALL_ID = 0x200,
	CAN_3508_M1_ID = 0x201,
	CAN_3508_M2_ID = 0x202,
	CAN_3508_M3_ID = 0x203, 
	CAN_3508_M4_ID = 0x204,
	CAN_3508_JC_ID = 0x205,
} can1_msg_id;

typedef enum
{
	DM_4310_M1_TX_ID = 0x01,
	MST_1_ID = 0x00,
	DM_YAW_TX_ID = 0x03,
	MST_YAW_ID = 0x02,
	CAN_2006_clamp_ID = 0x201,
} can2_msg_id;

typedef struct
{
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
	int16_t last_ecd;
	uint16_t count;
} motor_measure_t;

typedef struct
{
	uint8_t motor_err;
	uint8_t motor_tx_id;
	uint8_t motor_enabled;
	fp32 motor_position;
	fp32 motor_speed;
	fp32 motor_T;
}motor_DM_motor_t;

typedef enum
{
	CAN_COMMUNICATE_TX_ID_0 = 0x220,
	CAN_COMMUNICATE_TX_ID_1 = 0x221,
	CAN_COMMUNICATE_RX_ID_0 = 0x230,
	CAN_COMMUNICATE_RX_ID_1 = 0x231,
}can_board_communicate;


extern void CAN_cmd_chassis_reset_ID(void);
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_cmd_chassis_JC(int16_t motor1);
extern void CAN_cmd_chassis_clamp(int16_t motor1);
extern void CAN_cmd_4310_enable(uint8_t DM_ID, CAN_HandleTypeDef hcan);
extern void CAN_cmd_4310_disable(uint8_t DM_ID, CAN_HandleTypeDef hcan);
extern void CAN_cmd_4310_PV(fp32 DM_motor_position, fp32 DM_motor_speed, uint8_t DM_ID, CAN_HandleTypeDef hcan);
extern void CAN_cmd_4310_mit(fp32 DM_motor_position,fp32 DM_motor_speed,fp32 Kp,fp32 Kd,fp32 torq,uint8_t DM_ID, CAN_HandleTypeDef hcan);
extern const motor_measure_t *get_chassis_motor_point(uint8_t i);
extern motor_DM_motor_t *return_4310_measure(uint8_t i);
extern int float_to_uint(float x, float x_min, float x_max, int bits);
extern float uint_to_float(int x_int, float x_min, float x_max, int bits);

extern arm_communicate arm_message;
extern arm_communicate arm_message_temp;
#endif // !CAN_COMMUNICATE_H
