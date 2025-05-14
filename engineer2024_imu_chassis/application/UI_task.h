#ifndef _UI_TASK_H
#define _UI_TASK_H
#include "stm32f4xx_hal.h"
#include "referee.h"

//发送频率推荐不高于2Hz
#define UI_TIME 50

//内容ID data_cmd_id 长度（头结构长度+内容数据段长度）    功能说明
//0x0200~0x02FF      6+n                                  己方机器人间通信
//0x0100             6+2                                  客户端删除图形
//0x0101             6+15                                 客户端绘制一个图形
//0x0102             6+30                                 客户端绘制二个图形
//0x0103             6+75                                 客户端绘制五个图形
//0x0104             6+105                                客户端绘制七个图形
//0x0110             6+45                                 客户端绘制字符图

//机器人ID
typedef __packed struct
{
uint16_t data_cmd_id;					
uint16_t sender_ID;                   
uint16_t receiver_ID;
}ext_student_interactive_header_data_t;  

//图形数据
typedef __packed struct
{
	uint8_t graphic_name[3];
	uint32_t operate_type:3;
	uint32_t graphic_type:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t radius:10;
	uint32_t end_x:11;
	uint32_t end_y:11;
}graphic_data_struct_t;
//浮点型数据
typedef __packed struct
{
	uint8_t graphic_name[3];
	uint32_t operate_type:3;
	uint32_t graphic_type:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	int32_t    num;
}int_data_struct_t;   

//字符型数据
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
} ext_client_custom_character_t;
//绘制一个图形
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
}ext_client_custom_graphic_single_t;     
//绘制两个图形
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;
//绘制五个图形
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[5];
}ext_client_custom_graphic_five_t;        
//七图结构体
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[7];
}ext_client_custom_graphic_seven_t; 

//删除图形
typedef __packed struct
{
	uint8_t operate_tpye;         						
	uint8_t layer;
}ext_client_custom_graphic_delete_t;

//单图结构体
typedef __packed struct

{
	tFrameHeader                           FrameHead;                 //帧头
	uint16_t                               CmdId;                     //命令码 
	ext_student_interactive_header_data_t  Interactive_header_data;   //交互数据接收信息（数据段）
	ext_client_custom_graphic_single_t     Client_graphic_single;     //图形数据（数据段）
 	uint16_t                               CRC16;
}graphic_single_t;      
//两图结构体
typedef __packed struct
	
{
	tFrameHeader                           FrameHead;                 //帧头
	uint16_t                               CmdId;                     //命令码 
	ext_student_interactive_header_data_t  Interactive_header_data;   //交互数据接收信息（数据段）
	ext_client_custom_graphic_double_t     Client_graphic_double;     //图形数据（数据段）
 	uint16_t                               CRC16;
}graphic_double_t;      
//五图结构体
typedef __packed struct
{
	tFrameHeader                           FrameHead;                 //帧头
	uint16_t                               CmdId;                     //命令码 
	ext_student_interactive_header_data_t  Interactive_header_data;   //交互数据接收信息（数据段）
	ext_client_custom_graphic_five_t       Client_graphic_five;       //图形数据（数据段）
 	uint16_t                               CRC16;
}graphic_five_t;                        
//删除图层结构体
typedef __packed struct
{
	tFrameHeader                           FrameHead;                 //帧头
	uint16_t                               CmdId;                     //命令码 
	ext_student_interactive_header_data_t  Interactive_header_data;   //交互数据接收信息（数据段）
	ext_client_custom_graphic_delete_t     Client_Dele;               //图形数据（数据段）
 	uint16_t                               CRC16;
}graphic_dele_t;                      
//浮点数结构体
typedef __packed struct
{
	tFrameHeader                           FrameHead;              	//帧头
	uint16_t                               CmdId;                  	//命令码 
	ext_student_interactive_header_data_t  Interactive_header_data; //交互数据接收信息（数据段）
	int_data_struct_t                    Client_float_single;       //图形数据（数据段）
 	uint16_t                               CRC16;
}float_single_t;               
//字符型结构体
typedef __packed struct
{
	tFrameHeader                           FrameHead;             	//帧头
	uint16_t                               CmdId;                  	//命令码 
	ext_student_interactive_header_data_t  Interactive_header_data; //交互数据接收信息（数据段）
	ext_client_custom_character_t          Client_character;       	//图形数据（数据段）
 	uint16_t                               CRC16;
}character_t;                      
//七图结构体
typedef __packed struct
{
	tFrameHeader                           FrameHead;                 //帧头
	uint16_t                               CmdId;                     //命令码 
	ext_student_interactive_header_data_t  Interactive_header_data;   //交互数据接收信息（数据段）
	ext_client_custom_graphic_seven_t       Client_graphic_seven;       //图形数据（数据段）
 	uint16_t                               CRC16;
}graphic_seven_t;  

//UI信息
typedef __packed struct
{
	float_single_t float_single;
	graphic_five_t graphic_five;
	character_t    character;
}UI_message_t;	

extern void ui_task(void const *argu);

extern uint8_t opto_switch1_flag;
extern uint8_t opto_switch2_flag;

#endif
