#ifndef ARM_CONTROL_TASK_H
#define ARM_CONTROL_TASK_H

#include "struct_typedef.h"
#include "6dof_kinematic.h"
#include "Butterworth_Filter.h"
#include "TD.h"
#include "chassis_task.h"

typedef enum
{
	Ag1 = 0,
	Ag2 = 1,
	Ag3 = 2,
	EXCHANGE1 = 3,
	EXCHANGE2 = 4,
	Au1 = 5,
	Au2 = 6,
	Au3 = 7,
	NORMAL_POSITION = 8,
}arm_move_e;

typedef struct
{
	fp32 Ag1[11][6];
	fp32 Ag2[1][6];
	fp32 Ag3[10][6];
	fp32 exchange1[4][6];
	fp32 exchange2[4][6];
	fp32 Au1[9][6];
	fp32 Au2[10][6];
	fp32 Au3[4][6];
	int16_t flag_and_time[8][11][2];
}arm_routine_t;

typedef struct
{
	arm_routine_t arm_move_routine; //机械臂几种运动方式路径
	arm_move_e arm_move_flag; //机械臂运动方式标志位
	uint8_t arm_position_flag; //机械臂末端点位标志位
	uint16_t arm_get_position_flag; // 机械臂是否到达设定位置标志位
	
	fp32 one_key_position[6];
	fp32 repostion_position[6];
	fp32 gou_dong[6];
	fp32 three_ore_position[6];
	fp32 pre_Au_reposition[6];
	fp32 Au_reposition[6];
	fp32 Ag_reposition[6];
	uint8_t routine_length[9];
	
	fp32 motor_1_position;
	fp32 motor_2_position;
	fp32 motor_3_position;
	fp32 motor_4_position;
	fp32 motor_5_position;
	fp32 motor_6_position;
	
	motor_DM_t motor_YAW_data;
	
	fp32 lift_adjustion;
	float dt;
	uint32_t DWT_Count;

	TD_t arm_1_TD;
}arm_control_t;

typedef struct
{
	fp32 angleLimitMax;
	fp32 angleLimitMin;
}motor_t;

typedef struct
{

	int 			ValidCnt;
	float 		Joint_Valid[8][6];  //四组解在限制范围内的解
	float     Joint_Final[6];     //四组解中选择出来的最终解
//	float     high;
	Robotic_6DOF  Robotic_6D;
	
	Pose6D_t 			Pose6D_FK;//机械臂末端点位姿
	Joint6D_t 		Joint6D_FK;//机械臂各关节角度
	
	Pose6D_t 			Pose6D_IK;//机械臂目标位姿
	Solver6D_t 		output_solvers_IK;//机械臂逆解算输出结果（4组解）
	
	motor_t 			Joint[6];//关节最大最小范围限制
	
	TD_t Pose6D_IK_X_TD;
	TD_t Pose6D_IK_Y_TD;
	TD_t Pose6D_IK_Z_TD;
	
	
}Robotic_6DOF_control_t;

typedef struct
{
	fp32 Ag1;
	fp32 Ag2;
	fp32 Ag3;
}Ag_Catch_t;            //记录抓矿抬升初始位置
extern void arm_control_task(void const * argument);
extern arm_control_t arm_control;

extern uint8_t suker_key_flag;
//四元数模式（自定义控制器模式）
#define Quaterniont_Mode 1 
#define MaxJointSpeed 4.0f

#endif
