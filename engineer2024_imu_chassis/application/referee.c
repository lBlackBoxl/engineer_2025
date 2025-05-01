#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "protocol.h"

frame_header_struct_t referee_receive_header;
ext_game_robot_status_t robot_state;
ext_power_heat_data_t power_heat_data;
ext_shoot_data_t shoot_data;
ext_bullet_remaining_t bullet_remaining;
ext_robot_hurt_t robot_hurt;

uint8_t bullet_speed_corret_flag;
//与裁判系统通信初始化
void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&robot_state, 0, sizeof(ext_game_robot_status_t));
    memset(&power_heat_data, 0, sizeof(ext_power_heat_data_t));
	memset(&bullet_remaining, 0, sizeof(ext_bullet_remaining_t));
		memset(&robot_hurt, 0, sizeof(ext_robot_hurt_t));
}
//裁判系统数据解包
void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;
    uint8_t index = 0;
    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));
    index += sizeof(frame_header_struct_t);
    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);
    switch (cmd_id)
    {
        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&robot_state, frame + index, sizeof(ext_game_robot_status_t));
			break;
        }
        case POWER_HEAT_DATA_CMD_ID:
        {
			memcpy(&power_heat_data, frame + index, sizeof(ext_power_heat_data_t));
			break;
        }
        case SHOOT_DATA_CMD_ID:
        {
					  bullet_speed_corret_flag = 1;
            memcpy(&shoot_data, frame + index, sizeof(ext_shoot_data_t));
					  
			break;
        }
        case BULLET_REMAINING_CMD_ID:
        {
			memcpy(&bullet_remaining, frame + index, sizeof(ext_bullet_remaining_t));
			break;
					        case HURT_DATA_CMD_ID:
        {
			memcpy(&robot_hurt, frame + index, sizeof(ext_robot_hurt_t));
			break;
        }
        default:
        {
            break;
        }
    }
		}
}
