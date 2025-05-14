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

UI_message_t 	 	UI_message;				//UIͼ�νṹ��  //����ģʽ ��翪����X2  �������� ״̬����	3δʹ��	
UI_message_t 	 	UI_message_float;	//UI���ֽṹ��
graphic_dele_t 		message_dele;		//ɾ��ͼ��

uint8_t HOME[] = "HOME";
uint8_t Au[] = "Au";
uint8_t Ag[] = "Ag";
uint8_t Exchange[] = "Exchange";
uint8_t GouDong[] = "GouDong";

uint8_t NX_CONTROL[] = "NX_CONTROL";
uint8_t ONE_KEY[] = "ONE_KEY";
uint8_t SELF_CONTROL[] = "SELF_CONTROL";

uint8_t Error[] = "ERROR";

static void send_dele(uint8_t layer);
void Draw_layer_1(UI_message_t *UI_message, UI_message_t *UI_message_float, uint32_t operate_type);
void Draw_layer_2(UI_message_t *UI_message, UI_message_t *UI_message_float, uint32_t operate_type);
void Draw_layer_3(UI_message_t *UI_message, UI_message_t *UI_message_float, uint32_t operate_type);
void Draw_layer_4(UI_message_t *UI_message, UI_message_t *UI_message_float, uint32_t operate_type);
void Draw_layer_5(UI_message_t *UI_message, UI_message_t *UI_message_float, uint32_t operate_type);

//UI����
void ui_task(void const *argu)
{	
	osDelay(1000);
	//��ʼ��
	uint32_t UI_wake_time = osKernelSysTick();	
	
	while(1)
	{
		//Ϊ��ֹUI������ֹ��ϣ�ÿ��5s����һ������
		clear_time++;
		if(clear_time == 30)
		{
			for(int i = 0 ; i < 9; i++)
			{
				send_dele(i);
			}
			//�������ٽ����ʼ��
			Draw_layer_1(&UI_message,&UI_message_float,1);
			Draw_layer_2(&UI_message,&UI_message_float,1);
			Draw_layer_3(&UI_message,&UI_message_float,1);
			Draw_layer_4(&UI_message,&UI_message_float,1);
			Draw_layer_5(&UI_message,&UI_message_float,1);
			osDelay(UI_TIME);
			clear_time = 0;
		}
		
		//����UI��������
		Draw_layer_1(&UI_message,&UI_message_float,2);
		Draw_layer_2(&UI_message,&UI_message_float,2);
		Draw_layer_3(&UI_message,&UI_message_float,2);
		Draw_layer_4(&UI_message,&UI_message_float,2);
		Draw_layer_5(&UI_message,&UI_message_float,2);
		osDelay(UI_TIME);
	}
}

void Draw_layer_1(UI_message_t *UI_message, UI_message_t *UI_message_float, uint32_t operate_type)
{
	UI_message->graphic_five.FrameHead.sof = 0xA5;		
	UI_message->graphic_five.FrameHead.dataLenth = 81;
	UI_message->graphic_five.FrameHead.seq = (seqcount++) & 0xFF;  									//֡ͷ���
	UI_message->graphic_five.CmdId = 0x0301;                     									//����������
	UI_message->graphic_five.Interactive_header_data.data_cmd_id = 0x0103;   						//UIͼ��������
	UI_message->graphic_five.Interactive_header_data.sender_ID = robot_state.robot_id;				//�����˷���ID
	UI_message->graphic_five.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;	//�����˽���ID
	
	//������ʾ
	if(chassis_mode == 0)
	{
			//����ʱ��ʾ��ɫ
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 7;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 7;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].color = 7;
	}
	else if(chassis_mode == 1)
	{
			//����ʱ��ʾ��ɫ
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 2;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 2;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].color = 2;
	}
	else
	{
			//�쳣ʱ��ʾ��ɫ
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 3;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 3;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].color = 3;
	}
	//����
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[0] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[1] = 1;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[2] = 0;
	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].operate_type = operate_type;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_type = 1;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].layer = 1;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].width = 5;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_x = 800;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_y = 105;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_x = 800 + 320;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_y = 105 - 50;

	//����1
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[0] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[1] = 1;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[2] = 1;
	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].operate_type = operate_type;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_type = 2;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].layer = 1;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_angle = 0;				//��ʼ�Ƕ�	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].end_angle = 0;				//��ֹ�Ƕ�
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].width = 5;					//�߿�
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_x = 800 + 80;				//xԲ������
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_y = 105 - 50;				//yԲ������
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].radius = 25;					//Բ�뾶
	
	//����2
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[0] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[1] = 1;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[2] = 2;
	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].operate_type = operate_type;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_type = 2;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].layer = 1;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].start_angle = 0;				//��ʼ�Ƕ�	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].end_angle = 0;				//��ֹ�Ƕ�
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].width = 5;					//�߿�
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].start_x = 960 + 80;				//x�������
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].start_y = 105 - 50;				//y�������
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].radius = 25;					//Բ�뾶
	
	append_crc8_check_sum(&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five.FrameHead));
	append_crc16_check_sum(&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five.FrameHead)
	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message->graphic_five.FrameHead.dataLenth);
	HAL_UART_Transmit(&JUDGE_HUART,&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five),0xffff);
}

void Draw_layer_2(UI_message_t *UI_message, UI_message_t *UI_message_float, uint32_t operate_type)
{
	UI_message->graphic_five.FrameHead.sof = 0xA5;		
	UI_message->graphic_five.FrameHead.dataLenth = 81;
	UI_message->graphic_five.FrameHead.seq = (seqcount++) & 0xFF;  									//֡ͷ���
	UI_message->graphic_five.CmdId = 0x0301;                     									//����������
	UI_message->graphic_five.Interactive_header_data.data_cmd_id = 0x0103;   						//UIͼ��������
	UI_message->graphic_five.Interactive_header_data.sender_ID = robot_state.robot_id;				//�����˷���ID
	UI_message->graphic_five.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;	//�����˽���ID
	
	//��е����ʾ
	if(arm_restart_flag == 1 || chassis_mode == 0)
	{
			//�µ�or������ʾ��ɫ
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 7;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 7;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].color = 7;
	}
	else if(chassis_mode == 1)
	{
			//������ʾ��ɫ
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 2;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 2;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].color = 2;
	}
	else
	{
			//�쳣��ʾ��ɫ
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].color = 3;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 3;
			UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].color = 3;
	}
	
	//����1
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[0] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[1] = 2;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_name[2] = 0;
	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].operate_type = operate_type;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_type = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].layer = 2;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].width = 50;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_x = 960 - 80;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_y = 105;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_x = 960 + 80;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_y = 105 + 50;
	
	//�ؽ�
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[0] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[1] = 2;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[2] = 1;
	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].operate_type = operate_type;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_type = 2;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].layer = 2;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_angle = 0;				//��ʼ�Ƕ�	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].end_angle = 0;				//��ֹ�Ƕ�
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].width = 5;					//�߿�
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_x = 960 + 80;				//x�������
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_y = 105 + 50;				//y�������
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].radius = 30;					//Բ�뾶
	
	//����2
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[0] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[1] = 2;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[2] = 2;
	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].operate_type = operate_type;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_type = 1;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].layer = 2;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].width = 5;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].start_x = 960 + 80 - 100;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].start_y = 105 + 50 + 25;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].end_x = 960 + 80;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].end_y = 105 + 50 - 25;
	
	append_crc8_check_sum(&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five.FrameHead));
	append_crc16_check_sum(&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five.FrameHead)
	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message->graphic_five.FrameHead.dataLenth);
	HAL_UART_Transmit(&JUDGE_HUART,&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five),0xffff);
}

void Draw_layer_3(UI_message_t *UI_message, UI_message_t *UI_message_float, uint32_t operate_type)
{
	UI_message->graphic_five.FrameHead.sof = 0xA5;		
	UI_message->graphic_five.FrameHead.dataLenth = 81;
	UI_message->graphic_five.FrameHead.seq = (seqcount++) & 0xFF;  									//֡ͷ���
	UI_message->graphic_five.CmdId = 0x0301;                     									//����������
	UI_message->graphic_five.Interactive_header_data.data_cmd_id = 0x0103;   						//UIͼ��������
	UI_message->graphic_five.Interactive_header_data.sender_ID = robot_state.robot_id;				//�����˷���ID
	UI_message->graphic_five.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;	//�����˽���ID
	
	//�������
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

			//�����ʾ
	if(clamp_flag == 0)
	{
		//δ����ʱ������ͬɫ
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 8;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].color = 8;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].color = 8;
	}
	else if(clamp_flag == 1)
	{
		//��������ʾ��ɫ
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 2;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].color = 2;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].color = 2;
	}
	else
	{
		//�쳣��ʾ��ɫ
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].color = 3;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].color = 3;
		UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].color = 3;
	}
	
	//��ּ�ȡ����_����1
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[0] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[1] = 3;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_name[2] = 1;
	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].operate_type = operate_type;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_type = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].layer = 3;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].width = 20;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_x = 1550 + 75;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_y = 880;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].end_x = 1550 + 75;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].end_y = 880 - 40;
	
	//��ּ�ȡ����_����2
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[0] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[1] = 3;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_name[2] = 2;
	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].operate_type = operate_type;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_type = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].layer = 3;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].width = 20;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].start_x = 1550 + 300 - 75;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].start_y = 880;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].end_x = 1550 + 300 - 75;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].end_y = 880 - 40;

	//��ּ�ȡ����_�а�
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].graphic_name[0] = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].graphic_name[1] = 3;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].graphic_name[2] = 2;
	
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].operate_type = operate_type;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].graphic_type = 0;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].layer = 3;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].width = 10;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].start_x = 1550;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].start_y = 880 - 40;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].end_x = 1550 + 300;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[3].end_y = 880 - 40;
	
	append_crc8_check_sum(&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five.FrameHead));
	append_crc16_check_sum(&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five.FrameHead)
	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message->graphic_five.FrameHead.dataLenth);
	HAL_UART_Transmit(&JUDGE_HUART,&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five),0xffff);
}

void Draw_layer_4(UI_message_t *UI_message, UI_message_t *UI_message_float, uint32_t operate_type)
{
	UI_message->graphic_five.FrameHead.sof = 0xA5;		
	UI_message->graphic_five.FrameHead.dataLenth = 81;
	UI_message->graphic_five.FrameHead.seq = (seqcount++) & 0xFF;  									//֡ͷ���
	UI_message->graphic_five.CmdId = 0x0301;                     									//����������
	UI_message->graphic_five.Interactive_header_data.data_cmd_id = 0x0103;   						//UIͼ��������
	UI_message->graphic_five.Interactive_header_data.sender_ID = robot_state.robot_id;				//�����˷���ID
	UI_message->graphic_five.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;	//�����˽���ID
	
	//��ʯ1
	if(Ore_1_flag == 1)
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
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].graphic_type = 1;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].layer = 4;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].width = 5;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_x = 1550 + 35;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].start_y = 880 - 10;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_x = 1550 + 115;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[0].end_y = 880 - 90;

	//��ʯ2
	if(Ore_2_flag == 1)
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
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].graphic_type = 1;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].layer = 4;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].width = 5;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_x = 1550 + 300 - 35;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].start_y = 880 - 10;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].end_x = 1550 + 300 - 115;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[1].end_y = 880 - 90;
	
	//����״ָ̬ʾ
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
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].graphic_type = 1;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].layer = 4;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].width = 5;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].start_x = 1600;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].start_y = 580;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].end_x = 1800;
	UI_message->graphic_five.Client_graphic_five.grapic_data_struct[2].end_y = 530;
		
	append_crc8_check_sum(&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five.FrameHead));
	append_crc16_check_sum(&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five.FrameHead)
	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message->graphic_five.FrameHead.dataLenth);
	HAL_UART_Transmit(&JUDGE_HUART,&UI_message->graphic_five.FrameHead.sof,sizeof(UI_message->graphic_five),0xffff);
}

void Draw_layer_5(UI_message_t *UI_message, UI_message_t *UI_message_float, uint32_t operate_type)
{
	UI_message->graphic_five.FrameHead.sof = 0xA5;		
	UI_message->graphic_five.FrameHead.dataLenth = 81;
	UI_message->graphic_five.FrameHead.seq = (seqcount++) & 0xFF;  									//֡ͷ���
	UI_message->graphic_five.CmdId = 0x0301;                     									//����������
	UI_message->graphic_five.Interactive_header_data.data_cmd_id = 0x0103;   						//UIͼ��������
	UI_message->graphic_five.Interactive_header_data.sender_ID = robot_state.robot_id;				//�����˷���ID
	UI_message->graphic_five.Interactive_header_data.receiver_ID = 0x0100 + robot_state.robot_id;	//�����˽���ID
	
	//MOVE_MODE��ʾ
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
	memcpy(UI_message->character.Client_character.data, UI_String_1, sizeof(UI_String_1));
	UI_message->character.Client_character.grapic_data_struct.end_angle = sizeof(UI_String_1);
	
	append_crc8_check_sum(&UI_message->character.FrameHead.sof,sizeof(UI_message->character.FrameHead));
	append_crc16_check_sum(&UI_message->character.FrameHead.sof,sizeof(UI_message->character.FrameHead)
	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message->character.FrameHead.dataLenth);	
	HAL_UART_Transmit(&JUDGE_HUART,&UI_message->character.FrameHead.sof,sizeof(UI_message->character),0xffff);
	
	//MOVE_MODE״̬��ʾ
	uint8_t* UI_String_2;
	if(move_mode == 0)
	{
		UI_String_2 = HOME; 
		UI_message->character.Client_character.grapic_data_struct.color = 2;
	}
	else if(move_mode == 1)
	{
		UI_String_2 = Au;
		UI_message->character.Client_character.grapic_data_struct.color = 2;
	}
	else if(move_mode == 2)
	{
		UI_String_2 = Ag;
		UI_message->character.Client_character.grapic_data_struct.color = 2;
	}
	else if(move_mode == 3)
	{
		UI_String_2 = Exchange;
		UI_message->character.Client_character.grapic_data_struct.color = 2;
	}
	else if(move_mode == 4)
	{
		UI_String_2 = GouDong;
		UI_message->character.Client_character.grapic_data_struct.color = 2;
	}
	else
	{
		UI_String_2 = Error;
		UI_message->character.Client_character.grapic_data_struct.color = 3;
	}
	UI_message->character.Client_character.grapic_data_struct.graphic_name[0] = 0;
	UI_message->character.Client_character.grapic_data_struct.graphic_name[1] = 5;
	UI_message->character.Client_character.grapic_data_struct.graphic_name[2] = 1;
	UI_message->character.Client_character.grapic_data_struct.operate_type = operate_type;
	UI_message->character.Client_character.grapic_data_struct.graphic_type = 7;
	UI_message->character.Client_character.grapic_data_struct.layer = 5;
	UI_message->character.Client_character.grapic_data_struct.start_angle = 20;
	UI_message->character.Client_character.grapic_data_struct.end_angle = sizeof(UI_String_2) - 1;
	UI_message->character.Client_character.grapic_data_struct.width = 3;
	UI_message->character.Client_character.grapic_data_struct.start_x = 50;
	UI_message->character.Client_character.grapic_data_struct.start_y = 830;
	memcpy(UI_message->character.Client_character.data, UI_String_2, sizeof(UI_String_2) - 1);

	append_crc8_check_sum(&UI_message->character.FrameHead.sof,sizeof(UI_message->character.FrameHead));
	append_crc16_check_sum(&UI_message->character.FrameHead.sof,sizeof(UI_message->character.FrameHead)
	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message->character.FrameHead.dataLenth);	
	HAL_UART_Transmit(&JUDGE_HUART,&UI_message->character.FrameHead.sof,sizeof(UI_message->character),0xffff);
	
	//arm_mode��ʾ
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
	UI_message->character.Client_character.grapic_data_struct.start_y = 880;
	uint8_t UI_String_3[] = "ARM_MODE";
	memcpy(UI_message->character.Client_character.data, UI_String_3, sizeof(UI_String_3)-1);
	UI_message->character.Client_character.grapic_data_struct.end_angle = sizeof(UI_String_3) - 1;
	
	append_crc8_check_sum(&UI_message->character.FrameHead.sof,sizeof(UI_message->character.FrameHead));
	append_crc16_check_sum(&UI_message->character.FrameHead.sof,sizeof(UI_message->character.FrameHead)
	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message->character.FrameHead.dataLenth);	
	HAL_UART_Transmit(&JUDGE_HUART,&UI_message->character.FrameHead.sof,sizeof(UI_message->character),0xffff);
	
	//arm_mode״̬��ʾ
	uint8_t *UI_String_4;
	if(arm_mode == 0)
	{
		UI_String_4 = NX_CONTROL;
		UI_message->character.Client_character.grapic_data_struct.color = 2;
	}
	else if(arm_mode == 1)
	{
		UI_String_4 = ONE_KEY;
		UI_message->character.Client_character.grapic_data_struct.color = 2;
	}
	else if(arm_mode == 2)
	{
		UI_String_4 = SELF_CONTROL;
		UI_message->character.Client_character.grapic_data_struct.color = 2;
	}
	else
	{
		UI_String_4 = Error;
		UI_message->character.Client_character.grapic_data_struct.color = 3;
	}
	UI_message->character.Client_character.grapic_data_struct.graphic_name[0] = 0;
	UI_message->character.Client_character.grapic_data_struct.graphic_name[1] = 5;
	UI_message->character.Client_character.grapic_data_struct.graphic_name[2] = 3;
	UI_message->character.Client_character.grapic_data_struct.operate_type = operate_type;
	UI_message->character.Client_character.grapic_data_struct.graphic_type = 7;
	UI_message->character.Client_character.grapic_data_struct.layer = 5;
	UI_message->character.Client_character.grapic_data_struct.start_angle = 20;
	UI_message->character.Client_character.grapic_data_struct.end_angle = sizeof(UI_String_4) - 1;
	UI_message->character.Client_character.grapic_data_struct.width = 3;
	UI_message->character.Client_character.grapic_data_struct.start_x = 50;
	UI_message->character.Client_character.grapic_data_struct.start_y = 830;
	memcpy(UI_message->character.Client_character.data, UI_String_2, sizeof(UI_String_4) - 1);

	append_crc8_check_sum(&UI_message->character.FrameHead.sof,sizeof(UI_message->character.FrameHead));
	append_crc16_check_sum(&UI_message->character.FrameHead.sof,sizeof(UI_message->character.FrameHead)
	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message->character.FrameHead.dataLenth);	
	HAL_UART_Transmit(&JUDGE_HUART,&UI_message->character.FrameHead.sof,sizeof(UI_message->character),0xffff);
	
	//����״̬��ʾ
	UI_message->character.Client_character.grapic_data_struct.graphic_name[0] = 0;
	UI_message->character.Client_character.grapic_data_struct.graphic_name[1] = 5;
	UI_message->character.Client_character.grapic_data_struct.graphic_name[2] = 4;
	UI_message->character.Client_character.grapic_data_struct.operate_type = operate_type;
	UI_message->character.Client_character.grapic_data_struct.graphic_type = 7;
	UI_message->character.Client_character.grapic_data_struct.layer = 5;
	UI_message->character.Client_character.grapic_data_struct.color = 4;
	UI_message->character.Client_character.grapic_data_struct.start_angle = 20;
	UI_message->character.Client_character.grapic_data_struct.width = 3;
	UI_message->character.Client_character.grapic_data_struct.start_x = 50;
	UI_message->character.Client_character.grapic_data_struct.start_y = 880;
	uint8_t UI_String_5[] = "Suker_mode";
	memcpy(UI_message->character.Client_character.data, UI_String_5, sizeof(UI_String_5)-1);
	UI_message->character.Client_character.grapic_data_struct.end_angle = sizeof(UI_String_5) - 1;
	
	append_crc8_check_sum(&UI_message->character.FrameHead.sof,sizeof(UI_message->character.FrameHead));
	append_crc16_check_sum(&UI_message->character.FrameHead.sof,sizeof(UI_message->character.FrameHead)
	+ REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE + UI_message->character.FrameHead.dataLenth);	
	HAL_UART_Transmit(&JUDGE_HUART,&UI_message->character.FrameHead.sof,sizeof(UI_message->character),0xffff);
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
	HAL_UART_Transmit(&JUDGE_HUART,&message_dele.FrameHead.sof,sizeof(message_dele),0xffff);
}
