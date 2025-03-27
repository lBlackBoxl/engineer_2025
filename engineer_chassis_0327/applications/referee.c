#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "protocol.h"

frame_header_struct_t referee_receive_header;
ext_game_robot_status_t robot_state;
ext_rfid_status_t robot_rfid;
ext_arm_psoe_t    arm_pose;

//与裁判系统通信初始化
void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&robot_state, 0, sizeof(ext_game_robot_status_t));
    memset(&robot_rfid, 0, sizeof(ext_rfid_status_t));
    memset(&arm_pose, 0, sizeof(ext_arm_psoe_t));
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
				case ROBOT_RFID_CMD_ID:
				{
					memcpy(&robot_rfid, frame + index, sizeof(ext_rfid_status_t));
					break;
				}
				case ROBOT_POSE_CMD_ID:
				{
					memcpy(&arm_pose, frame + index, sizeof(ext_arm_psoe_t));
					break;
				}
        default:
        {
            break;
        }
    }
}
