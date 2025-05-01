#ifndef REFEREE_H
#define REFEREE_H
#include "main.h"
typedef enum
{
    ROBOT_STATE_CMD_ID                = 0x0201,
		ROBOT_RFID_CMD_ID                 = 0x0209,
		ROBOT_POSE_CMD_ID                 = 0x0302
}referee_cmd_id_t;

typedef __packed struct
{
    uint8_t  sof;
    uint16_t dataLenth;
    uint8_t  seq;
    uint8_t  crc8;
} tFrameHeader;
typedef __packed struct //0x0201
{

 uint8_t robot_id;
 uint8_t robot_level;
 uint16_t remain_HP;
 uint16_t max_HP;
 uint16_t shooter_id1_17mm_cooling_rate;
 uint16_t shooter_id1_17mm_cooling_limit;
 uint16_t shooter_id1_17mm_speed_limit;
 uint16_t shooter_id2_17mm_cooling_rate;
 uint16_t shooter_id2_17mm_cooling_limit;
 uint16_t shooter_id2_17mm_speed_limit;
 uint16_t shooter_id1_42mm_cooling_rate;
 uint16_t shooter_id1_42mm_cooling_limit;
 uint16_t shooter_id1_42mm_speed_limit;
 uint16_t chassis_power_limit;
 uint8_t mains_power_gimbal_output : 1;
 uint8_t mains_power_chassis_output : 1;
 uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;


typedef __packed struct
{
	uint32_t rfid_status;
} ext_rfid_status_t;

typedef __packed struct
{
	float x;
	float y;
	float z;
	float q[4];
	uint16_t nothing;
} ext_arm_psoe_t;

extern ext_arm_psoe_t    arm_pose;
extern ext_game_robot_status_t robot_state;
extern ext_rfid_status_t robot_rfid;

extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);
#endif
