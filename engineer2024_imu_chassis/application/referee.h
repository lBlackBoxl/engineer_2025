#ifndef REFEREE_H
#define REFEREE_H
#include "main.h"
typedef enum
{
    ROBOT_STATE_CMD_ID                = 0x0201,
    POWER_HEAT_DATA_CMD_ID            = 0x0202,
		HURT_DATA_CMD_ID					=0X0206,
    SHOOT_DATA_CMD_ID                 = 0x0207,
    BULLET_REMAINING_CMD_ID           = 0x0208,
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

typedef __packed struct //0x0202
{
	uint16_t chassis_volt;
	uint16_t chassis_current;
	float chassis_power;
	uint16_t chassis_power_buffer;
	uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id2_17mm_cooling_heat;
	uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;
typedef __packed struct//0x207 
{   
	uint8_t bullet_type;  
	uint8_t shooter_id;   
	uint8_t bullet_freq;    
	float bullet_speed;  
} ext_shoot_data_t; 
typedef __packed struct//0x208
{
	uint16_t bullet_remaining_num_17mm;   
	uint16_t bullet_remaining_num_42mm; 
	uint16_t coin_remaining_num; 
} ext_bullet_remaining_t;
/**
   @brief …À∫¶◊¥Ã¨£∫0x0206£¨ ‹µΩ…À∫¶∫Û∑¢ÀÕ
*/
typedef struct {
  uint8_t armor_id : 4;
  uint8_t hurt_type : 4;
} ext_robot_hurt_t;



extern ext_game_robot_status_t robot_state;
extern ext_power_heat_data_t power_heat_data;
extern ext_shoot_data_t shoot_data;
extern ext_bullet_remaining_t bullet_remaining;
extern ext_robot_hurt_t robot_hurt;

extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);
#endif
