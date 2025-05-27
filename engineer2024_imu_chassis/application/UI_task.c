#include "UI_task.h"
#include "bsp_usart.h"
#include "CAN_receive.h"
#include "usart.h"
#include "stdio.h"
#include "protocol.h"
#include "cmsis_os.h"
//#include "ore_task.h"
//#include "catch_task.h"
#include "usart_communicate.h"
#include "string.h"

uint8_t seqcount = 0;
uint8_t clear_time = 0;
uint8_t opto_switch1_flag;
uint8_t opto_switch2_flag;

UI_message_t 	 	UI_message;				//UI图形结构体  //底盘模式 光电开关线X2  底盘轮廓 状态文字	3未使用	
UI_message_t 	 	UI_message_float;	//UI数字结构体
graphic_dele_t 		message_dele;		//删除图层
uint8_t Temp[128];
extern uint8_t last_move_mode;
extern uint8_t last_arm_mode;
extern uint8_t last_clamp_flag;
uint8_t restart_flag;

static void send_dele(uint8_t layer);
void Draw_layer_0(UI_message_t *UI_message, UI_message_t *UI_message_float, uint32_t operate_type);
void Draw_layer_1(UI_message_t *UI_message, UI_message_t *UI_message_float, uint32_t operate_type);
void Draw_layer_2(UI_message_t *UI_message, UI_message_t *UI_message_float, uint32_t operate_type);
void Draw_layer_3(UI_message_t *UI_message, UI_message_t *UI_message_float, uint32_t operate_type);
void Draw_layer_4(UI_message_t *UI_message, UI_message_t *UI_message_float, uint32_t operate_type);
void Draw_layer_5(UI_message_t *UI_message, UI_message_t *UI_message_float, uint32_t operate_type);
void Draw_layer_6(UI_message_t *UI_message, UI_message_t *UI_message_float, uint32_t operate_type);

//UI任务
void ui_task(void const *argu)
{	
	osDelay(2000);
	//初始化
	uint32_t UI_wake_time = osKernelSysTick();	
	
    for(int i = 0 ; i < 9; i++)
    {
        send_dele(i);
        osDelay(UI_TIME);
    }
	Draw_layer_1(&UI_message,&UI_message_float,1);
	osDelay(UI_TIME);
	Draw_layer_2(&UI_message,&UI_message_float,1);
	osDelay(UI_TIME);
	Draw_layer_3(&UI_message,&UI_message_float,1);
	osDelay(UI_TIME);
	Draw_layer_4(&UI_message,&UI_message_float,1);
	osDelay(UI_TIME);
	Draw_layer_5(&UI_message,&UI_message_float,1);
	osDelay(UI_TIME);
	
	while(1)
	{
		//为防止UI界面出现故障，每隔10s进行一次清屏
		if((last_arm_mode != arm_mode) || (last_move_mode != move_mode) || (last_clamp_flag != clamp_flag))
		{
				restart_flag = 1;
		}
		else
		{
				restart_flag = 0;
		}
		
		clear_time++;
		if(clear_time == 200 || restart_flag == 1)
		{
			//清屏
			if(last_clamp_flag != clamp_flag)
			{
				send_dele(3);
				osDelay(UI_TIME);
				Draw_layer_3(&UI_message,&UI_message_float,1);
				osDelay(UI_TIME);
			}
			else
			{
				for(int i = 0 ; i < 9; i++)
				{
					send_dele(i);
									osDelay(UI_TIME);
				}
				//清屏后再进入初始化
							if(arm_mode == 2)
							{

							}
							else
							{
									if(move_mode == 4)
									{
											Draw_layer_0(&UI_message,&UI_message_float,1);
											osDelay(UI_TIME);
									}
									else if(move_mode == 2)
									{
											Draw_layer_6(&UI_message,&UI_message_float,1);
											osDelay(UI_TIME);
									}
									Draw_layer_1(&UI_message,&UI_message_float,1);
									osDelay(UI_TIME);
									Draw_layer_2(&UI_message,&UI_message_float,1);
									osDelay(UI_TIME);
									Draw_layer_3(&UI_message,&UI_message_float,1);
									osDelay(UI_TIME);
									Draw_layer_4(&UI_message,&UI_message_float,1);
									osDelay(UI_TIME);
									Draw_layer_5(&UI_message,&UI_message_float,1);
									osDelay(UI_TIME);
									clear_time = 0;
							}
				}
		}
		
		last_arm_mode = arm_mode;
		last_move_mode = move_mode;
		last_clamp_flag = clamp_flag;
		
		//更新UI界面内容
		if(arm_mode == 2)
		{

		}
		else
		{
				if(move_mode == 4)
				{
					Draw_layer_0(&UI_message,&UI_message_float,2);
								osDelay(UI_TIME);
				}
				else if(move_mode == 2)
				{
					Draw_layer_6(&UI_message,&UI_message_float,2);
					osDelay(UI_TIME);
				}
				Draw_layer_1(&UI_message,&UI_message_float,2);
						osDelay(UI_TIME);
				Draw_layer_2(&UI_message,&UI_message_float,2);
						osDelay(UI_TIME);
				Draw_layer_3(&UI_message,&UI_message_float,2);
						osDelay(UI_TIME);
				Draw_layer_4(&UI_message,&UI_message_float,2);
						osDelay(UI_TIME);
				Draw_layer_5(&UI_message,&UI_message_float,2);
				osDelay(UI_TIME);
		}
	}
}
void Draw_layer_0(UI_message_t *UI_message, UI_message_t *UI_message_float, uint32_t operate_type)
{
	UI_message->graphic_five.FrameHead.sof = 0xA5;		
	UI_message->graphic_five.FrameHead.dataLenth = 81;
	UI_message->graphic_five.FrameHead.seq = (seqcount++) & 0xFF;  									//帧头填充
	UI_message->graphic_five.CmdId = 0x0301;                     									//交互命令码
	UI_message->graphic_five.Interactive_header_data.data_cmd_id = 0x0103;   						//UI图形命令码
	UI_message->graphic_five.Interactive_header_data.sender_ID = robot_state.robot_id;				//机器人发送ID
	UI_message->graphic_five.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;	//机器人接收ID
	
	//狗洞辅助线
	//辅助线1
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[0] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[1] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[2] = 0;
	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].operate_type = operate_type;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_type = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].layer = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].width = 3;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 4;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_x = 800;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_y = 720;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_x = 680;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_y = 360;
	
	//辅助线2
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[0] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[1] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[2] = 1;
	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].operate_type = operate_type;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_type = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].layer = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].width = 3;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 4;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_x = 1080;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_y = 720;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].end_x = 1220;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].end_y = 360;
	
	append_crc8_check_sum(&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five.FrameHead));
	append_crc16_check_sum(&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five.FrameHead)
	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message->graphic_five.FrameHead.dataLenth);
	HAL_UART_Transmit_DMA(&JUDGE_HUART,&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five));
	
}
void Draw_layer_1(UI_message_t *UI_message, UI_message_t *UI_message_float, uint32_t operate_type)
{
	UI_message->graphic_five.FrameHead.sof = 0xA5;		
	UI_message->graphic_five.FrameHead.dataLenth = 81;
	UI_message->graphic_five.FrameHead.seq = (seqcount++) & 0xFF;  									//帧头填充
	UI_message->graphic_five.CmdId = 0x0301;                     									//交互命令码
	UI_message->graphic_five.Interactive_header_data.data_cmd_id = 0x0103;   						//UI图形命令码
	UI_message->graphic_five.Interactive_header_data.sender_ID = robot_state.robot_id;				//机器人发送ID
	UI_message->graphic_five.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;	//机器人接收ID
	
	//底盘显示
	if(chassis_mode == 0)
	{
			//无力时显示黑色
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 7;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 7;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].color = 7;
	}
	else if(chassis_mode == 1)
	{
			//上力时显示绿色
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 2;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 2;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].color = 2;
	}
	else
	{
			//异常时显示橙色
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 3;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 3;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].color = 3;
	}
	//车体
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[0] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[1] = 1;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[2] = 0;
	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].operate_type = operate_type;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_type = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].layer = 1;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].width = 50;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_x = 800;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_y = 80;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_x = 800 + 320;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_y = 80;

	//车轮1
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[0] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[1] = 1;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[2] = 1;
	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].operate_type = operate_type;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_type = 2;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].layer = 1;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_angle = 0;				//起始角度	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].end_angle = 0;				//终止角度
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].width = 5;					//线宽
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_x = 800 + 80;				//x圆心坐标
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_y = 105 - 50;				//y圆心坐标
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].radius = 25;					//圆半径
	
	//车轮2
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[0] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[1] = 1;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[2] = 2;
	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].operate_type = operate_type;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_type = 2;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].layer = 1;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].start_angle = 0;				//起始角度	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].end_angle = 0;				//终止角度
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].width = 5;					//线宽
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].start_x = 960 + 80;				//x起点坐标
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].start_y = 105 - 50;				//y起点坐标
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].radius = 25;					//圆半径
	
	append_crc8_check_sum(&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five.FrameHead));
	append_crc16_check_sum(&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five.FrameHead)
	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message->graphic_five.FrameHead.dataLenth);
    memcmp(Temp, &UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five));
	HAL_UART_Transmit_DMA(&JUDGE_HUART,&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five));
}

void Draw_layer_2(UI_message_t *UI_message, UI_message_t *UI_message_float, uint32_t operate_type)
{
	UI_message->graphic_five.FrameHead.sof = 0xA5;		
	UI_message->graphic_five.FrameHead.dataLenth = 81;
	UI_message->graphic_five.FrameHead.seq = (seqcount++) & 0xFF;  									//帧头填充
	UI_message->graphic_five.CmdId = 0x0301;                     									//交互命令码
	UI_message->graphic_five.Interactive_header_data.data_cmd_id = 0x0103;   						//UI图形命令码
	UI_message->graphic_five.Interactive_header_data.sender_ID = robot_state.robot_id;				//机器人发送ID
	UI_message->graphic_five.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;	//机器人接收ID
	
	//机械臂显示
	if(arm_restart_flag == 1 || chassis_mode == 0)
	{
			//下电or下力显示黑色
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 7;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 7;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].color = 7;
	}
	else if(chassis_mode == 1)
	{
			//上力显示绿色
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 2;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 2;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].color = 2;
	}
	else if(arm_error_flag == 1)
	{
			//异常显示橙色
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 3;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 3;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].color = 3;
	}
	else
	{
			//异常显示橙色
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 3;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 3;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].color = 3;
	}
	
	//连杆1
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[0] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[1] = 2;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[2] = 0;
	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].operate_type = operate_type;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_type = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].layer = 2;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].width = 36;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_x = 960 - 80;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_y = 85;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_x = 960 + 80;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_y = 105 + 80;
	
	//关节
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[0] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[1] = 2;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[2] = 1;
	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].operate_type = operate_type;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_type = 2;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].layer = 2;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_angle = 0;				//起始角度	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].end_angle = 0;				//终止角度
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].width = 5;					//线宽
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_x = 960 + 80;				//x起点坐标
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_y = 105 + 80;				//y起点坐标
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].radius = 30;					//圆半径
	
	//连杆2
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[0] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[1] = 2;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[2] = 2;
	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].operate_type = operate_type;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_type = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].layer = 2;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].width = 36;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].start_x = 960 + 80;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].start_y = 105 + 80;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].end_x = 960 + 80 - 150;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].end_y = 105 + 80;
	
	append_crc8_check_sum(&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five.FrameHead));
	append_crc16_check_sum(&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five.FrameHead)
	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message->graphic_five.FrameHead.dataLenth);
    
	HAL_UART_Transmit_DMA(&JUDGE_HUART,&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five));
}

void Draw_layer_3(UI_message_t *UI_message, UI_message_t *UI_message_float, uint32_t operate_type)
{
	UI_message->graphic_five.FrameHead.sof = 0xA5;		
	UI_message->graphic_five.FrameHead.dataLenth = 81;
	UI_message->graphic_five.FrameHead.seq = (seqcount++) & 0xFF;  									//帧头填充
	UI_message->graphic_five.CmdId = 0x0301;                     									//交互命令码
	UI_message->graphic_five.Interactive_header_data.data_cmd_id = 0x0103;   						//UI图形命令码
	UI_message->graphic_five.Interactive_header_data.sender_ID = robot_state.robot_id;				//机器人发送ID
	UI_message->graphic_five.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;	//机器人接收ID
	
	//矿仓主体
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[0] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[1] = 3;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[2] = 0;
	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].operate_type = operate_type;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_type = 1;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 8;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].layer = 3;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].width = 5;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_x = 1550;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_y = 880;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_x = 1550 + 300;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_y = 880 - 100;

	//矿仓显示
	if(clamp_flag == 1)
	{
		//开启后显示绿色
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 2;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].color = 2;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].color = 2;
		
		//矿仓夹取机关_连杆1
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[0] = 0;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[1] = 3;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[2] = 1;
		
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].operate_type = operate_type;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_type = 0;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].layer = 3;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].width = 20;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_x = 1550 + 75;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_y = 880 - 100;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].end_x = 1550 + 75;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].end_y = 880 - 100 - 40;
	
		//矿仓夹取机关_连杆2
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[0] = 0;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[1] = 3;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[2] = 2;
		
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].operate_type = operate_type;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_type = 0;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].layer = 3;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].width = 20;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].start_x = 1550 + 300 - 75;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].start_y = 880 - 100;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].end_x = 1550 + 300 - 75;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].end_y = 880 - 100 - 40;

		//矿仓夹取机关_夹板
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].graphic_name[0] = 0;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].graphic_name[1] = 3;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].graphic_name[2] = 3;
		
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].operate_type = operate_type;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].graphic_type = 0;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].layer = 3;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].width = 10;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].start_x = 1550;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].start_y = 880 - 100 - 40;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].end_x = 1550 + 300;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].end_y = 880 - 100 - 40;
	}
	
	append_crc8_check_sum(&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five.FrameHead));
	append_crc16_check_sum(&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five.FrameHead)
	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message->graphic_five.FrameHead.dataLenth);
	HAL_UART_Transmit_DMA(&JUDGE_HUART,&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five));
}

void Draw_layer_4(UI_message_t *UI_message, UI_message_t *UI_message_float, uint32_t operate_type)
{
	UI_message->graphic_five.FrameHead.sof = 0xA5;		
	UI_message->graphic_five.FrameHead.dataLenth = 81;
	UI_message->graphic_five.FrameHead.seq = (seqcount++) & 0xFF;  									//帧头填充
	UI_message->graphic_five.CmdId = 0x0301;                     									//交互命令码
	UI_message->graphic_five.Interactive_header_data.data_cmd_id = 0x0103;   						//UI图形命令码
	UI_message->graphic_five.Interactive_header_data.sender_ID = robot_state.robot_id;				//机器人发送ID
	UI_message->graphic_five.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;	//机器人接收ID
	
	//矿石1
	if(Ore_1_flag == 0)
	{
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 1;
	}
	else 
	{
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 8;
	}
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[0] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[1] = 4;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[2] = 0;
	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].operate_type = operate_type;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_type = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].layer = 4;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].width = 80;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_x = 1550 + 35;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_y = 880 - 50;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_x = 1550 + 115;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_y = 880 - 50;

	//矿石2
	if(Ore_2_flag == 0)
	{
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 1;
	}
	else
	{
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 8;
	}
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[0] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[1] = 4;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[2] = 1;
	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].operate_type = operate_type;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_type = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].layer = 4;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].width = 80;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_x = 1550 + 300 - 35;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_y = 880 - 50;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].end_x = 1550 + 300 - 115;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].end_y = 880 - 50;

	//气泵状态指示
	if(suker_key_flag == 1)
	{
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].color = 2;
	}
	else
	{
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].color = 8;
	}
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[0] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[1] = 4;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[2] = 2;
	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].operate_type = operate_type;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_type = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].layer = 4;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].width = 50;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].start_x = 1600;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].start_y = 600;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].end_x = 1800;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].end_y = 600;
		
	append_crc8_check_sum(&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five.FrameHead));
	append_crc16_check_sum(&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five.FrameHead)
	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message->graphic_five.FrameHead.dataLenth);
	HAL_UART_Transmit_DMA(&JUDGE_HUART,&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five));
}

void Draw_layer_5(UI_message_t *UI_message, UI_message_t *UI_message_float, uint32_t operate_type)
{
	UI_message->character.FrameHead.sof = 0xA5;		
	UI_message->character.FrameHead.dataLenth = 51;
	UI_message->character.FrameHead.seq = (seqcount++) & 0xFF;  									//帧头填充
	UI_message->character.CmdId = 0x0301;                     									//交互命令码
	UI_message->character.Interactive_header_data.data_cmd_id = 0x0110;   						//UI图形命令码
	UI_message->character.Interactive_header_data.sender_ID = robot_state.robot_id;				//机器人发送ID
	UI_message->character.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;	//机器人接收ID
	
	//MOVE_MODE显示
	UI_message->character.Client_character.grapic_data_struct.graphic_name[0] = 0;
	UI_message->character.Client_character.grapic_data_struct.graphic_name[1] = 5;
	UI_message->character.Client_character.grapic_data_struct.graphic_name[2] = 0;
	UI_message->character.Client_character.grapic_data_struct.operate_type = operate_type;
	UI_message->character.Client_character.grapic_data_struct.graphic_type = 7;
	UI_message->character.Client_character.grapic_data_struct.layer = 5;
	UI_message->character.Client_character.grapic_data_struct.color = 4;
	UI_message->character.Client_character.grapic_data_struct.start_angle = 20;
	UI_message->character.Client_character.grapic_data_struct.width = 3;
	UI_message->character.Client_character.grapic_data_struct.start_x = 50;
	UI_message->character.Client_character.grapic_data_struct.start_y = 880;
	uint8_t UI_String_1[] = {'M','o','v','e','_','m','o','d','e'};
    memset(UI_message->character.Client_character.data, 0, sizeof(UI_message->character.Client_character));
	memcpy(UI_message->character.Client_character.data, UI_String_1, sizeof(UI_String_1));
	UI_message->character.Client_character.grapic_data_struct.end_angle = sizeof(UI_String_1);
	
	append_crc8_check_sum(&UI_message->character.FrameHead.sof,sizeof(UI_message->character.FrameHead));
	append_crc16_check_sum(&UI_message->character.FrameHead.sof,sizeof(UI_message->character.FrameHead)
	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message->character.FrameHead.dataLenth);	
	HAL_UART_Transmit_DMA(&JUDGE_HUART,&UI_message->character.FrameHead.sof,sizeof(UI_message->character));
    osDelay(UI_TIME);
	
	//MOVE_MODE状态显示
	if(move_mode == 0)
	{
		uint8_t UI_String_2[] = "HOME"; 
		UI_message->character.Client_character.grapic_data_struct.color = 2;
        UI_message->character.Client_character.grapic_data_struct.end_angle = sizeof(UI_String_2);
        memset(UI_message->character.Client_character.data, 0, sizeof(UI_message->character.Client_character));
        memcpy(UI_message->character.Client_character.data, UI_String_2, sizeof(UI_String_2));
	}
	else if(move_mode == 1)
	{
		uint8_t UI_String_2[] = "Au"; 
		UI_message->character.Client_character.grapic_data_struct.color = 2;
        UI_message->character.Client_character.grapic_data_struct.end_angle = sizeof(UI_String_2);
        memset(UI_message->character.Client_character.data, 0, sizeof(UI_message->character.Client_character));
        memcpy(UI_message->character.Client_character.data, UI_String_2, sizeof(UI_String_2));
	}
	else if(move_mode == 2)
	{
		uint8_t UI_String_2[] = "Ag"; 
		UI_message->character.Client_character.grapic_data_struct.color = 2;
        UI_message->character.Client_character.grapic_data_struct.end_angle = sizeof(UI_String_2);
        memset(UI_message->character.Client_character.data, 0, sizeof(UI_message->character.Client_character));
        memcpy(UI_message->character.Client_character.data, UI_String_2, sizeof(UI_String_2));
	}
	else if(move_mode == 3)
	{
		uint8_t UI_String_2[] = "EXCHANGE"; 
		UI_message->character.Client_character.grapic_data_struct.color = 2;
        UI_message->character.Client_character.grapic_data_struct.end_angle = sizeof(UI_String_2);
        memset(UI_message->character.Client_character.data, 0, sizeof(UI_message->character.Client_character));
        memcpy(UI_message->character.Client_character.data, UI_String_2, sizeof(UI_String_2));
	}
	else if(move_mode == 4)
	{
		uint8_t UI_String_2[] = "GouDong"; 
		UI_message->character.Client_character.grapic_data_struct.color = 2;
        UI_message->character.Client_character.grapic_data_struct.end_angle = sizeof(UI_String_2);
        memset(UI_message->character.Client_character.data, 0, sizeof(UI_message->character.Client_character));
        memcpy(UI_message->character.Client_character.data, UI_String_2, sizeof(UI_String_2));
	}
    else if(move_mode == 5)
    {
		uint8_t UI_String_2[] = "WAIT"; 
		UI_message->character.Client_character.grapic_data_struct.color = 2;
        UI_message->character.Client_character.grapic_data_struct.end_angle = sizeof(UI_String_2);
        memset(UI_message->character.Client_character.data, 0, sizeof(UI_message->character.Client_character));
        memcpy(UI_message->character.Client_character.data, UI_String_2, sizeof(UI_String_2));
    }
	else
	{
		uint8_t UI_String_2[] = "ERROR"; 
		UI_message->character.Client_character.grapic_data_struct.color = 3;
        UI_message->character.Client_character.grapic_data_struct.end_angle = sizeof(UI_String_2);
        memset(UI_message->character.Client_character.data, 0, sizeof(UI_message->character.Client_character));
        memcpy(UI_message->character.Client_character.data, UI_String_2, sizeof(UI_String_2));
	}
	UI_message->character.Client_character.grapic_data_struct.graphic_name[0] = 0;
	UI_message->character.Client_character.grapic_data_struct.graphic_name[1] = 5;
	UI_message->character.Client_character.grapic_data_struct.graphic_name[2] = 1;
	UI_message->character.Client_character.grapic_data_struct.operate_type = operate_type;
	UI_message->character.Client_character.grapic_data_struct.graphic_type = 7;
	UI_message->character.Client_character.grapic_data_struct.layer = 5;
	UI_message->character.Client_character.grapic_data_struct.start_angle = 20;
	UI_message->character.Client_character.grapic_data_struct.width = 3;
	UI_message->character.Client_character.grapic_data_struct.start_x = 50;
	UI_message->character.Client_character.grapic_data_struct.start_y = 830;

	append_crc8_check_sum(&UI_message->character.FrameHead.sof,sizeof(UI_message->character.FrameHead));
	append_crc16_check_sum(&UI_message->character.FrameHead.sof,sizeof(UI_message->character.FrameHead)
	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message->character.FrameHead.dataLenth);	
	HAL_UART_Transmit_DMA(&JUDGE_HUART,&UI_message->character.FrameHead.sof,sizeof(UI_message->character));
    osDelay(UI_TIME);
    
	//arm_mode显示
	UI_message->character.Client_character.grapic_data_struct.graphic_name[0] = 0;
	UI_message->character.Client_character.grapic_data_struct.graphic_name[1] = 5;
	UI_message->character.Client_character.grapic_data_struct.graphic_name[2] = 2;
	UI_message->character.Client_character.grapic_data_struct.operate_type = operate_type;
	UI_message->character.Client_character.grapic_data_struct.graphic_type = 7;
	UI_message->character.Client_character.grapic_data_struct.layer = 5;
	UI_message->character.Client_character.grapic_data_struct.color = 4;
	UI_message->character.Client_character.grapic_data_struct.start_angle = 20;
	UI_message->character.Client_character.grapic_data_struct.width = 3;
	UI_message->character.Client_character.grapic_data_struct.start_x = 50;
	UI_message->character.Client_character.grapic_data_struct.start_y = 780;
	uint8_t UI_String_3[] = "ARM_MODE";
    memset(UI_message->character.Client_character.data, 0, sizeof(UI_message->character.Client_character));
	memcpy(UI_message->character.Client_character.data, UI_String_3, sizeof(UI_String_3)-1);
	UI_message->character.Client_character.grapic_data_struct.end_angle = sizeof(UI_String_3) - 1;
	
	append_crc8_check_sum(&UI_message->character.FrameHead.sof,sizeof(UI_message->character.FrameHead));
	append_crc16_check_sum(&UI_message->character.FrameHead.sof,sizeof(UI_message->character.FrameHead)
	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message->character.FrameHead.dataLenth);	
	HAL_UART_Transmit_DMA(&JUDGE_HUART,&UI_message->character.FrameHead.sof,sizeof(UI_message->character));
    osDelay(UI_TIME);
	
	//arm_mode状态显示
	if(arm_mode == 0)
	{
		uint8_t UI_String_4[] = "NX_CONTROL";
		UI_message->character.Client_character.grapic_data_struct.color = 2;
        UI_message->character.Client_character.grapic_data_struct.end_angle = sizeof(UI_String_4);
        memset(UI_message->character.Client_character.data, 0, sizeof(UI_message->character.Client_character));
        memcpy(UI_message->character.Client_character.data, UI_String_4, sizeof(UI_String_4));
    }
	else if(arm_mode == 1)
	{
		uint8_t UI_String_4[] = "ONE_KEY_MODE";
		UI_message->character.Client_character.grapic_data_struct.color = 2;
        UI_message->character.Client_character.grapic_data_struct.end_angle = sizeof(UI_String_4);
        memset(UI_message->character.Client_character.data, 0, sizeof(UI_message->character.Client_character));
        memcpy(UI_message->character.Client_character.data, UI_String_4, sizeof(UI_String_4));
	}
	else if(arm_mode == 2)
	{
		uint8_t UI_String_4[] = "SELF_CONTROL";
		UI_message->character.Client_character.grapic_data_struct.color = 2;
        UI_message->character.Client_character.grapic_data_struct.end_angle = sizeof(UI_String_4);
        memset(UI_message->character.Client_character.data, 0, sizeof(UI_message->character.Client_character));
        memcpy(UI_message->character.Client_character.data, UI_String_4, sizeof(UI_String_4));
	}
	else
	{
		uint8_t UI_String_4[] = "ERROR";
		UI_message->character.Client_character.grapic_data_struct.color = 3;
        UI_message->character.Client_character.grapic_data_struct.end_angle = sizeof(UI_String_4);
        memset(UI_message->character.Client_character.data, 0, sizeof(UI_message->character.Client_character));
        memcpy(UI_message->character.Client_character.data, UI_String_4, sizeof(UI_String_4));
	}
	UI_message->character.Client_character.grapic_data_struct.graphic_name[0] = 0;
	UI_message->character.Client_character.grapic_data_struct.graphic_name[1] = 5;
	UI_message->character.Client_character.grapic_data_struct.graphic_name[2] = 3;
	UI_message->character.Client_character.grapic_data_struct.operate_type = operate_type;
	UI_message->character.Client_character.grapic_data_struct.graphic_type = 7;
	UI_message->character.Client_character.grapic_data_struct.layer = 5;
	UI_message->character.Client_character.grapic_data_struct.start_angle = 20;
	UI_message->character.Client_character.grapic_data_struct.width = 3;
	UI_message->character.Client_character.grapic_data_struct.start_x = 50;
	UI_message->character.Client_character.grapic_data_struct.start_y = 730;

	append_crc8_check_sum(&UI_message->character.FrameHead.sof,sizeof(UI_message->character.FrameHead));
	append_crc16_check_sum(&UI_message->character.FrameHead.sof,sizeof(UI_message->character.FrameHead)
	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message->character.FrameHead.dataLenth);	
	HAL_UART_Transmit_DMA(&JUDGE_HUART,&UI_message->character.FrameHead.sof,sizeof(UI_message->character));
    osDelay(UI_TIME);
	
	//气泵状态显示
	UI_message->character.Client_character.grapic_data_struct.graphic_name[0] = 0;
	UI_message->character.Client_character.grapic_data_struct.graphic_name[1] = 5;
	UI_message->character.Client_character.grapic_data_struct.graphic_name[2] = 4;
	UI_message->character.Client_character.grapic_data_struct.operate_type = operate_type;
	UI_message->character.Client_character.grapic_data_struct.graphic_type = 7;
	UI_message->character.Client_character.grapic_data_struct.layer = 5;
	UI_message->character.Client_character.grapic_data_struct.color = 4;
	UI_message->character.Client_character.grapic_data_struct.start_angle = 20;
	UI_message->character.Client_character.grapic_data_struct.width = 3;
	UI_message->character.Client_character.grapic_data_struct.start_x = 1600;
	UI_message->character.Client_character.grapic_data_struct.start_y = 680;
	uint8_t UI_String_5[] = "Suker_mode";
    memset(UI_message->character.Client_character.data, 0, sizeof(UI_message->character.Client_character));
	memcpy(UI_message->character.Client_character.data, UI_String_5, sizeof(UI_String_5)-1);
	UI_message->character.Client_character.grapic_data_struct.end_angle = sizeof(UI_String_5) - 1;
	
	append_crc8_check_sum(&UI_message->character.FrameHead.sof,sizeof(UI_message->character.FrameHead));
	append_crc16_check_sum(&UI_message->character.FrameHead.sof,sizeof(UI_message->character.FrameHead)
	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message->character.FrameHead.dataLenth);	
	HAL_UART_Transmit_DMA(&JUDGE_HUART,&UI_message->character.FrameHead.sof,sizeof(UI_message->character));
}

void Draw_layer_6(UI_message_t *UI_message, UI_message_t *UI_message_float, uint32_t operate_type)
{
	UI_message->graphic_five.FrameHead.sof = 0xA5;		
	UI_message->graphic_five.FrameHead.dataLenth = 81;
	UI_message->graphic_five.FrameHead.seq = (seqcount++) & 0xFF;  									//帧头填充
	UI_message->graphic_five.CmdId = 0x0301;                     									//交互命令码
	UI_message->graphic_five.Interactive_header_data.data_cmd_id = 0x0103;   						//UI图形命令码
	UI_message->graphic_five.Interactive_header_data.sender_ID = robot_state.robot_id;				//机器人发送ID
	UI_message->graphic_five.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;	//机器人接收ID
	
	//银矿辅助线
	//辅助线1
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[0] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[1] = 6;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[2] = 0;
	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].operate_type = operate_type;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_type = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].layer = 6;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].width = 3;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 4;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_x = 700;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_y = 160;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_x = 660;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_y = 0;
	
	//辅助线2 
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[0] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[1] = 6;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[2] = 1;
	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].operate_type = operate_type;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_type = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].layer = 6;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].width = 3;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 4;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_x = 700;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_y = 160;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_x = 980;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_y = 140;
	
	//辅助线3
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[0] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[1] = 6;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[2] = 2;
	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].operate_type = operate_type;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_type = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].layer = 6;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].width = 3;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 4;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_x = 1240;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_y = 160;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_x = 1300;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_y = 0;
	
	//辅助线4
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[0] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[1] = 6;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[2] = 3;
	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].operate_type = operate_type;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_type = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].layer = 6;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].width = 3;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 4;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_x = 1240;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_y = 160;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_x = 980;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_y = 140;
	
	append_crc8_check_sum(&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five.FrameHead));
	append_crc16_check_sum(&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five.FrameHead)
	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message->graphic_five.FrameHead.dataLenth);
	HAL_UART_Transmit_DMA(&JUDGE_HUART,&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five));
}

static void send_dele(uint8_t lay)
{
	message_dele.FrameHead.sof = 0xA5;
	message_dele.FrameHead.dataLenth = 8;
	message_dele.FrameHead.seq = (seqcount++) & 0xFF;
	message_dele.CmdId = 0x0301;     
	message_dele.Interactive_header_data.data_cmd_id = 0x0100;    
	message_dele.Interactive_header_data.sender_ID = robot_state.robot_id;
	message_dele.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;
	message_dele.Client_Dele.operate_tpye = 1;
	message_dele.Client_Dele.layer = lay;	
	
	append_crc8_check_sum(&message_dele.FrameHead.sof,sizeof(message_dele.FrameHead));
	append_crc16_check_sum(&message_dele.FrameHead.sof,sizeof(message_dele.FrameHead)
	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + message_dele.FrameHead.dataLenth);
	HAL_UART_Transmit_DMA(&JUDGE_HUART,&message_dele.FrameHead.sof,sizeof(message_dele));
}
