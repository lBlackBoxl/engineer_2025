#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "protocol.h"
#include "keyboard.h"

frame_header_struct_t referee_receive_header;
ext_game_robot_status_t robot_state;
ext_rfid_status_t robot_rfid;
#if SELF_CTRL_XYZYPR	
ext_arm_pose_t    arm_pose;
#else
ext_arm_position_t	arm_position;
#endif
uint8_t orientation_flag;
orientation_station orientation_mode;
orientation_station last_orientation_mode;
key_t Rocker_key;

//与裁判系统通信初始化
void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&robot_state, 0, sizeof(ext_game_robot_status_t));
    memset(&robot_rfid, 0, sizeof(ext_rfid_status_t));
#if SELF_CTRL_XYZYPR	
    memset(&arm_pose, 0, sizeof(ext_arm_pose_t));
#else
    memset(&arm_position, 0, sizeof(ext_arm_position_t));
#endif
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
#if	SELF_CTRL_XYZYPR
					memcpy(&arm_pose, frame + index, sizeof(ext_arm_pose_t));
#else
					memcpy(&arm_position, frame + index, sizeof(ext_arm_position_t));
					Rocker_key.itself.last_mode = Rocker_key.itself.mode;
					if(Rocker_key.itself.flag == 0)
					{
						if(arm_position.Rocker_button != 0)
						{
								Rocker_key.itself.time++;
						}
						if(Rocker_key.itself.time >= 1) 
						{	
							Rocker_key.itself.flag = 1;
							Rocker_key.itself.time = 0;
						}
					}
					else                                        
					{
						if(arm_position.Rocker_button == 0)   
						{
								Rocker_key.itself.time++;
						}
						if(Rocker_key.itself.time >= 1) 
						{	
								Rocker_key.itself.flag = 0;
								Rocker_key.itself.time = 0;
								Rocker_key.itself.mode = Rocker_key.itself.mode +1;
								if(Rocker_key.itself.mode == 2)
								Rocker_key.itself.mode=0;
						}
					}
					
#endif
					break;
				}
        default:
        {
            break;
        }
    }
}
