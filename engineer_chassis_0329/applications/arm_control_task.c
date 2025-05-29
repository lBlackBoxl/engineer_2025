#include "arm_control_task.h"
#include "chassis_task.h"
#include "can_communicate.h"
#include "keyboard.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "referee.h"
#include "bsp_dwt.h"
#include "math.h"
#include "gpio.h"
#include "can_communicate.h"
#include "init.h"
#include "nx_communicate.h"
#include "can.h"
#include "6dof_kinematic.h"

//为了兼容两个模式，增加了一个参数isomorphic_control_t* i_ctrl
//SELF_CTRL_XYZYPR == 1 时不使用该参数
static void arm_control_init(all_key_t *arm_control_key_init, Robotic_6DOF_control_t *R_6D_ctrl);
static void arm_feedback_update(arm_control_t *arm_control_position, Robotic_6DOF_control_t *R_6D_ctrl, isomorphic_control_t* i_ctrl);//此处加了参数
static void arm_control_key_check(all_key_t *arm_cnotrol_key_check);
static void arm_control_set(arm_control_t *arm_control_set, all_key_t *arm_key);
static void arm_check_get_position(arm_control_t *check_position);
static void arm_control_loop(Robotic_6DOF_control_t *R_6D_ctrl, arm_control_t *arm_control_loop, all_key_t *arm_key, isomorphic_control_t* i_ctrl);//此处加了参数

arm_control_t arm_control;
Robotic_6DOF_control_t R_6D_ctrl;
//SELF_CTRL_XYZYPR == 1 时不使用该变量
isomorphic_control_t  I_6D_ctrl;
Ag_Catch_t Ag_Catch;

float max_joint_speed[6];
Joint6D_t Last_Joint6D;
uint8_t suker_key_flag;
uint8_t AJX_flag;
float32_t yaw_angle_set;
fp32 arm_target_position[6];
uint16_t time_flag[8][2][2];
uint16_t time_cnt = 0;
uint16_t arm_error_cnt = 0;
uint8_t error_flag = 0;
fp32 sc_allowance[2] = {0.0f, 0.0f};
extern bool_t  clamp_flag;
int Half_rod_cnt;
fp32 allowance[6] = {-0.0f, -0.1f, 0.0f, 0.0f, 0.06f, 0.0f};

void arm_control_task(void const *argument)
{
	arm_control_init(&all_key, &R_6D_ctrl);
	Joint6D_Init(&R_6D_ctrl.Robotic_6D);

	uint32_t system_clock = osKernelSysTick();
	while (1)
	{
		arm_control.dt = DWT_GetDeltaT(&arm_control.DWT_Count);

		//SELF_CTRL_XYZYPR == 1 时不使用参数&I_6D_ctrl
		arm_feedback_update(&arm_control, &R_6D_ctrl, &I_6D_ctrl);
		arm_control_key_check(&all_key);
		arm_control_set(&arm_control, &all_key);
		//SELF_CTRL_XYZYPR == 1 时不使用参数&I_6D_ctrl
		arm_control_loop(&R_6D_ctrl, &arm_control, &all_key, &I_6D_ctrl);
		osDelay(2);
		CAN_cmd_4310_mit(arm_control.motor_YAW_data.position_set, 0.0f, 1500.0f, 2.0f, 0.0f, DM_YAW_TX_ID, hcan2);
		DWT_Delay(0.0003f);
	}
}

void arm_control_init(all_key_t *arm_control_key_init, Robotic_6DOF_control_t *R_6D_ctrl)
{
	//默认给机械臂上电
	HAL_GPIO_WritePin(Arm_Power_GPIO_Port,Arm_Power_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Yaw_Power_GPIO_Port, Yaw_Power_Pin, GPIO_PIN_SET);
	
	//默认不启用AJX一键
	AJX_flag = 0;
	
	//AJX时间标志位初始化
	time_flag[Ag1][0][0] = 1, time_flag[Ag1][0][1] = 1000, time_flag[Ag1][1][0] = 0, time_flag[Ag1][1][1] = 2000;
	time_flag[Ag2][0][0] = 1, time_flag[Ag2][0][1] = 1000, time_flag[Ag2][1][0] = 0, time_flag[Ag2][1][1] = 2000;	
	time_flag[Ag3][0][0] = 1, time_flag[Ag3][0][1] = 1000, time_flag[Ag3][1][0] = 0, time_flag[Ag3][1][1] = 2000;
	time_flag[Au1][0][0] = 1, time_flag[Au1][0][1] = 1000, time_flag[Au1][1][0] = 0, time_flag[Au1][1][1] = 2000;
	time_flag[Au2][0][0] = 1, time_flag[Au2][0][1] = 1000, time_flag[Au2][1][0] = 0, time_flag[Au2][1][1] = 2000;
	time_flag[Au3][0][0] = 1, time_flag[Au3][0][1] = 1000, time_flag[Au3][1][0] = 0, time_flag[Au3][1][1] = 2000;
	time_flag[EXCHANGE1][0][0] = 1, time_flag[EXCHANGE1][0][1] = 1000, time_flag[EXCHANGE1][1][0] = 1, time_flag[EXCHANGE1][1][1] = 1000;
	time_flag[EXCHANGE2][0][0] = 1, time_flag[EXCHANGE2][0][1] = 1000, time_flag[EXCHANGE2][1][0] = 1, time_flag[EXCHANGE2][1][1] = 1000;

	// YAW关节电机初始化
	arm_control.motor_YAW_data.DM_motor_measure = return_4310_measure(1);
	CAN_cmd_4310_disable(DM_YAW_TX_ID, hcan2);
	DWT_Delay(0.0003f);

	arm_control.arm_move_flag = NORMAL_POSITION;
	arm_control.arm_position_flag = 0;
	arm_control.arm_get_position_flag = 0;
	arm_control.motor_1_position = 0.0f;
	arm_control.motor_2_position = 0.0f;
	arm_control.motor_3_position = 0.0f;
	arm_control.motor_4_position = 0.0f;
	arm_control.motor_5_position = 0.0f;
	arm_control.motor_6_position = 0.0f;
	
#if SELF_CTRL_XYZYPR
	arm_pose.x = 3.0f;
	arm_pose.y = 0.0f;
	arm_pose.z = 0.0f; 
	arm_pose.q[0] = 1.0f;
	arm_pose.q[1] = 0.0f;
	arm_pose.q[2] = 0.0f;
	arm_pose.q[3] = 0.0f;
#else
	memset(&arm_position, 0, sizeof(arm_position));
#endif

	key_init(&arm_control_key_init->capture_key, Z);
	key_init(&arm_control_key_init->suker_key, R);
	key_init(&arm_control_key_init->ajx_on_key, F);
	key_init(&arm_control_key_init->self_control_rotate1_key,SHIFT);
	key_init(&arm_control_key_init->self_control_rotate2_key,CTRL);
	// 各关节角度限制
	R_6D_ctrl->Joint[0].angleLimitMax = +1.57f;
	R_6D_ctrl->Joint[0].angleLimitMin = -1.57f;

	R_6D_ctrl->Joint[1].angleLimitMax = -0.329f; // 3.1f;
	R_6D_ctrl->Joint[1].angleLimitMin = -3.129f; // 0.14f;

	R_6D_ctrl->Joint[2].angleLimitMax = 1.558f;	 // 1.57f;
	R_6D_ctrl->Joint[2].angleLimitMin = -1.242f; //-1.23f;

	R_6D_ctrl->Joint[3].angleLimitMax = 2.35f;//1.87f;	///////2.3f;
	R_6D_ctrl->Joint[3].angleLimitMin = -2.35f;//-1.87f; ///////-1.9f;

	R_6D_ctrl->Joint[4].angleLimitMax = PI;
	R_6D_ctrl->Joint[4].angleLimitMin = -PI;

	R_6D_ctrl->Joint[5].angleLimitMax = PI;
	R_6D_ctrl->Joint[5].angleLimitMin = -PI;

	for (int i = 0; i < 5; i++)
	{
		R_6D_ctrl->Joint_Final[i] = 0.0f;
	}
	//		uint8_t routine_length[9] = {12, 12, 10, 6, 7, 10, 10 ,3};

	uint8_t routine_length[9] = {11, 11, 6, 5, 5, 10, 10, 4};

	for (int i = 0; i < 9; i++)
	{
		arm_control.routine_length[i] = routine_length[i];
	}

	//		TD_init(&R_6D_ctrl->Pose6D_IK_Z_TD, 50.0f, 2.0f, 0.002f, R_6D_ctrl->Pose6D_IK.Z);

	fp32 Ag1_temp[11][6] = {
													{0.708f + allowance[0], 0.72f + allowance[1], 0.64f + allowance[2], 0.0f + allowance[3], 1.8f + allowance[4], 0.0f + allowance[5]},
													{0.344f + allowance[0], 1.62f + allowance[1], 0.61f + allowance[2], 0.0f + allowance[3], 2.43f + allowance[4], 0.0f + allowance[5]},
													{0.344f + allowance[0], 1.75f + allowance[1], 0.62f + allowance[2], 0.0f + allowance[3], 2.43f + allowance[4], 0.0f + allowance[5]},
													{0.344f + allowance[0], 1.3f + allowance[1], 0.70f + allowance[2], 0.0f + allowance[3], 2.93f + allowance[4], 0.0f + allowance[5]},
													{0.344f + allowance[0], 0.96f + allowance[1], 0.96f + allowance[2], 0.0f + allowance[3], 3.30f + allowance[4], 0.0f + allowance[5]},
													{-0.641f + allowance[0], 0.72f + allowance[1], 0.69f + allowance[2], 0.0f + allowance[3], 3.30f + allowance[4], 0.0f + allowance[5]},
													{-2.656f + allowance[0], 1.12f + allowance[1], 0.87f + allowance[2], 0.0f + allowance[3], 2.90f + allowance[4], -0.35f + allowance[5]},
													{-2.656f + allowance[0], 0.99f + allowance[1], 0.68f + allowance[2], 0.0f + allowance[3], 2.90f + allowance[4], -0.35f + allowance[5]},
													{-2.656f + allowance[0], 0.99f + allowance[1], 0.68f + allowance[2], 0.0f + allowance[3], 2.90f + allowance[4], -0.35f + allowance[5]},
													{-0.641f + allowance[0], 1.0f + allowance[1], 0.96f + allowance[2], 0.0f + allowance[3], 3.30f + allowance[4], 0.0f + allowance[5]},
													{0.708f + allowance[0], 0.92f + allowance[1], 0.64f + allowance[2], 0.0f + allowance[3], 1.8f + allowance[4], 0.0f + allowance[5]},
												};

	uint16_t flag_and_time_1[11][2] = {{0, 50}, {1, 1000}, {1, 200}, {1, 200}, {1, 200}, {1, 50}, {1, 100}, {1,500}, {0,500},{0, 50}, {0, 50}};
	Ag_Catch.Ag1 = Ag1_temp[3][0];
	memcpy(arm_control.arm_move_routine.Ag1, Ag1_temp, sizeof(Ag1_temp));
	memcpy(arm_control.arm_move_routine.flag_and_time[0], flag_and_time_1, sizeof(flag_and_time_1));

	fp32 Ag2_temp[11][6] = {
													{0.708f + allowance[0], 0.72f + allowance[1], 0.64f + allowance[2], 0.0f + allowance[3], 1.8f + allowance[4], 0.0f + allowance[5]},
													{1.144f + allowance[0], 1.62f + allowance[1], 0.61f + allowance[2], 0.0f + allowance[3], 2.43f + allowance[4], 0.0f + allowance[5]},
													{1.144f + allowance[0], 1.75f + allowance[1], 0.62f + allowance[2], 0.0f + allowance[3], 2.43f + allowance[4], 0.0f + allowance[5]},
													{1.144f + allowance[0], 1.3f + allowance[1], 0.70f + allowance[2], 0.0f + allowance[3], 2.93f + allowance[4], 0.0f + allowance[5]},
													{1.114f + allowance[0], 0.96f + allowance[1], 0.96f + allowance[2], 0.0f + allowance[3], 3.30f + allowance[4], 0.0f + allowance[5]},
													{-1.456f + allowance[0], 1.26f + allowance[1], 1.10f + allowance[2], 0.0f + allowance[3], 3.05f + allowance[4], 0.0f + allowance[5]},
													{-2.216f + allowance[0], 1.22f + allowance[1], 0.93f + allowance[2], 0.0f + allowance[3], 3.30f + allowance[4], -0.22f + allowance[5]},
													{-2.216f + allowance[0], 1.22f + allowance[1], 0.93f + allowance[2], 0.0f + allowance[3], 3.30f + allowance[4], -0.22f + allowance[5]},
													{-2.216f + allowance[0], 1.22f + allowance[1], 0.93f + allowance[2], 0.0f + allowance[3], 3.30f + allowance[4], -0.22f + allowance[5]},
													{-0.641f + allowance[0], 1.0f + allowance[1], 0.96f + allowance[2], 0.0f + allowance[3], 3.30f + allowance[4], 0.0f + allowance[5]},
													{0.708f + allowance[0], 0.92f + allowance[1], 0.64f + allowance[2], 0.0f + allowance[3], 1.8f + allowance[4], 0.0f + allowance[5]},
												};

	uint16_t flag_and_time_2[11][2] = {{0, 50}, {1, 1000}, {1, 200}, {1, 200}, {1, 200}, {1, 50}, {1, 100}, {1, 500}, {0,500},{0, 50}, {0, 50}};
	Ag_Catch.Ag2 = Ag2_temp[3][0];
	memcpy(arm_control.arm_move_routine.Ag2,Ag2_temp,sizeof(Ag2_temp));
	memcpy(arm_control.arm_move_routine.flag_and_time[1], flag_and_time_2,sizeof(flag_and_time_2));
	
	fp32 Ag3_temp[6][6] = {	{0.708f + allowance[0], 0.72f + allowance[1], 0.64f + allowance[2], 0.0f + allowance[3], 1.8f + allowance[4], -0.57f + allowance[5]},
													{1.144f + allowance[0], 1.62f + allowance[1], 0.61f + allowance[2], 0.0f + allowance[3], 2.43f + allowance[4], -0.57f + allowance[5]},
													{1.144f + allowance[0], 1.75f + allowance[1], 0.62f + allowance[2], 0.0f + allowance[3], 2.43f + allowance[4], -0.57f + allowance[5]},
													{1.144f + allowance[0], 1.3f + allowance[1], 0.70f + allowance[2], 0.0f + allowance[3], 2.93f + allowance[4], -0.57f + allowance[5]},
													{1.144f + allowance[0], 0.96f + allowance[1], 0.96f + allowance[2], 0.0f + allowance[3], 3.30f + allowance[4], 0.0f + allowance[5]},			
													{0.708f + allowance[0], 0.72f + allowance[1], 0.64f + allowance[2], 0.0f + allowance[3], 1.8f + allowance[4], 0.0f + allowance[5]}};

	uint16_t flag_and_time_3[6][2] = {{0,50},{1,1000},{1,200},{1,200},{1,200},{1,200}};
	memcpy(arm_control.arm_move_routine.Ag3,Ag3_temp,sizeof(Ag3_temp));
	memcpy(arm_control.arm_move_routine.flag_and_time[2], flag_and_time_3,sizeof(flag_and_time_3));
	Ag_Catch.Ag3 = 	Ag3_temp[3][0];									
	
//	fp32 exchange1_temp[4][6] = {{-0.785f, 0.72f, 0.64f, 0.0f, 3.30f, 0.0f},
//													{-1.65f,0.4f,0.0f,1.364f,0.72f,1.3f},
//													{-1.75f,0.72f,0.1f,1.30f,0.72f,1.3f},
//													{0.564f,0.72f,0.64f,0.0f,1.8f,0.0f}
//													};
//	
//	uint16_t flag_and_time_4[4][2] = {{0,200},{1,500},{1,200},{1,1000}};
	
	fp32 exchange1_temp[5][6] = {	{-0.641f + allowance[0], 0.92f + allowance[1], 0.64f + allowance[2], 0.0f + allowance[3], 3.0f + allowance[4], 0.0f + allowance[5]},
																{-2.038f + allowance[0],1.3f + allowance[1],0.9f + allowance[2],-0.1f + allowance[3],3.0f + allowance[4],-0.87f + allowance[5]},
																{-2.038f + allowance[0],1.2f + allowance[1],0.60f + allowance[2],-0.1f + allowance[3],3.10f + allowance[4],-0.85f + allowance[5]},
																{-2.038f + allowance[0],1.3f + allowance[1],0.9f + allowance[2],-0.1f + allowance[3],3.10f + allowance[4],-0.85f + allowance[5]},
																{0.708f + allowance[0],1.3f + allowance[1],1.0f + allowance[2],0.0f + allowance[3],1.8f + allowance[4],0.0f + allowance[5]}
															};
	
	uint16_t flag_and_time_4[5][2] = {{0,200},{1,200},{1,200},{1,500},{1,200}};
	
	memcpy(arm_control.arm_move_routine.exchange1,exchange1_temp,sizeof(exchange1_temp));
	memcpy(arm_control.arm_move_routine.flag_and_time[3], flag_and_time_4,sizeof(flag_and_time_4));
	
	fp32 exchange2_temp[5][6] = {{-0.641f + allowance[0], 0.92f + allowance[1], 0.64f + allowance[2], 0.0f + allowance[3], 3.0f + allowance[4], 0.0f + allowance[5]},
													{-2.69f + allowance[0], 1.15f + allowance[1], 0.75f + allowance[2],0.02f + allowance[3], 3.20f + allowance[4], 0.0f + allowance[5]},
													{-2.69f + allowance[0], 1.05f + allowance[1], 0.50f + allowance[2],0.02f + allowance[3], 3.20f + allowance[4], 0.0f + allowance[5]},
													{-2.69f + allowance[0], 1.15f + allowance[1], 0.75f + allowance[2],0.02f + allowance[3], 3.20f + allowance[4], 0.0f + allowance[5]},
													{0.708f + allowance[0],0.92f + allowance[1],1.2f + allowance[2],0.0f + allowance[3],1.8f + allowance[4],0.0f + allowance[5]}};
	
	uint16_t flag_and_time_5[5][2] = {{0,200},{1,200},{1,200},{1,200},{1,500}};
	
	memcpy(arm_control.arm_move_routine.exchange2,exchange2_temp,sizeof(exchange2_temp));
	memcpy(arm_control.arm_move_routine.flag_and_time[4], flag_and_time_5,sizeof(flag_and_time_5));
	
	fp32 Au1_temp[10][6] = {{0.708f + allowance[0], 2.60f + allowance[1], 2.35f + allowance[2], 0.0f + allowance[3], 1.53f + allowance[4], 0.0f + allowance[5]},
													{0.708f + allowance[0], 2.60f + allowance[1], 2.35f + allowance[2], 0.0f + allowance[3], 1.53f + allowance[4], 0.0f + allowance[5]},
													{0.708f + allowance[0], 2.43f + allowance[1], 2.26f + allowance[2], 0.0f + allowance[3], 1.98f + allowance[4], 0.0f + allowance[5]},
													{0.708f + allowance[0], 0.72f + allowance[1], 0.64f + allowance[2], 0.0f + allowance[3], 1.85f + allowance[4], 0.0f + allowance[5]},
													{-0.641f + allowance[0], 0.20f + allowance[1], 0.2f + allowance[2], 0.0f + allowance[3], 1.36f + allowance[4], 0.0f + allowance[5]},
													{-2.87f + allowance[0], 0.20f + allowance[1], 0.2f + allowance[2], -0.67f + allowance[3], 1.02f + allowance[4], 1.2f + allowance[5]},
													{-2.87f + allowance[0], 0.64f + allowance[1], 0.1f + allowance[2], -0.67f + allowance[3], 1.02f + allowance[4], 1.2f + allowance[5]},
													{-2.87f + allowance[0], 0.64f + allowance[1], 0.1f + allowance[2], -0.67f + allowance[3], 1.02f + allowance[4], 1.2f + allowance[5]},
													{-2.87f + allowance[0], 0.06f + allowance[1], 0.6f + allowance[2], -0.67f + allowance[3], 1.02f + allowance[4], 1.2f + allowance[5]},
													{0.708f + allowance[0], 0.94f + allowance[1], 0.6f + allowance[2], 0.0f + allowance[3], 1.8f + allowance[4], 0.0f + allowance[5]},
												 };
	uint16_t flag_and_time_6[10][2] = {{0,300},{1,300},{1,1500},{1,50},{1,50},{1,50},{1,100},{0,500},{0,500},{0,100}};
	
	memcpy(arm_control.arm_move_routine.Au1,Au1_temp,sizeof(Au1_temp));	  
	memcpy(arm_control.arm_move_routine.flag_and_time[5], flag_and_time_6,sizeof(flag_and_time_6));
	
	fp32 Au2_temp[10][6] = {	{0.708f + allowance[0], 2.60f + allowance[1], 2.35f + allowance[2], 0.0f + allowance[3], 1.53f + allowance[4], 0.0f + allowance[5]},
                                {0.708f + allowance[0], 2.60f + allowance[1], 2.35f + allowance[2], 0.0f + allowance[3], 1.53f + allowance[4], 0.0f + allowance[5]},
                                {0.708f + allowance[0], 2.43f + allowance[1], 2.26f + allowance[2], 0.0f + allowance[3], 1.98f + allowance[4], 0.0f + allowance[5]},
                                {0.708f + allowance[0], 0.82f + allowance[1], 0.64f + allowance[2], 0.0f + allowance[3], 1.98f + allowance[4], 0.0f + allowance[5]},
                                {-0.641f + allowance[0], 0.82f + allowance[1], 0.6f + allowance[2], 0.0f + allowance[3], 1.36f + allowance[4], 0.0f + allowance[5]},
                                {-1.846f + allowance[0], 0.74f + allowance[1], 0.5f + allowance[2], -2.19f + allowance[3], 2.72f + allowance[4], -0.86f + allowance[5]},
                                {-1.936f + allowance[0], 0.79f + allowance[1], 0.16f + allowance[2], -2.19f + allowance[3], 2.72f + allowance[4], -0.86f + allowance[5]},
                                {-1.936f + allowance[0], 0.79f + allowance[1], 0.16f + allowance[2], -2.19f + allowance[3], 2.72f + allowance[4], -0.86f + allowance[5]},
                                {-0.641f + allowance[0], 0.79f + allowance[1], 0.16f + allowance[2], 0.0f + allowance[3], 1.80f + allowance[4], 0.0f + allowance[5]},
                                {0.708f + allowance[0], 0.92f + allowance[1], 0.5f + allowance[2], 0.0f + allowance[3], 1.80f + allowance[4], 0.0f + allowance[5]},};          //复位  放下
	
	uint16_t flag_and_time_7[10][2] = {{0,1500},{1,300},{1,1500},{1,50},{1,50},{1,50},{1,100},{0,500},{0,50},{0,500}};
	memcpy(arm_control.arm_move_routine.Au2,Au2_temp,sizeof(Au2_temp));	
	memcpy(arm_control.arm_move_routine.flag_and_time[6], flag_and_time_7,sizeof(flag_and_time_7));
	
	fp32 Au3_temp[4][6] = {{0.708f + allowance[0], 2.60f + allowance[1], 2.35f + allowance[2], 0.0f + allowance[3], 1.53f + allowance[4], 0.0f + allowance[5]},
														{0.708f + allowance[0], 2.60f + allowance[1], 2.35f + allowance[2], 0.0f + allowance[3], 1.53f + allowance[4], 0.0f + allowance[5]},
														{0.708f + allowance[0], 2.43f + allowance[1], 2.26f + allowance[2], 0.0f + allowance[3], 1.98f + allowance[4], 0.0f + allowance[5]},
														{0.708f + allowance[0], 2.37f + allowance[1], 2.20f + allowance[2], 0.0f + allowance[3], 1.98f + allowance[4], 0.0f + allowance[5]},   
												};
	
	uint16_t flag_and_time_8[4][2] = {{0,200},{1,1000},{1,300},{1,300}};
	
	memcpy(arm_control.arm_move_routine.Au3,Au3_temp,sizeof(Au3_temp));	
	memcpy(arm_control.arm_move_routine.flag_and_time[7], flag_and_time_8,sizeof(flag_and_time_8));

	// 机械臂复位位置
	fp32 repositon_position[6] = {0.708f + allowance[0], /*0.72f*/0.67f + allowance[1], 0.5f + allowance[2], 0.0f + allowance[3], 1.8f + allowance[4], 0.0f + allowance[5]};
	fp32 gou_dong[6] = {0.708f + allowance[0], 2.87f + allowance[1], 2.54f + allowance[2], 0.0f + allowance[3], 0.2f + allowance[4], 0.0f + allowance[5]};
	fp32 three_ore_position[6] = {0.708f + allowance[0], 0.72f + allowance[1], 0.5f + allowance[2], 0.0f + allowance[3], 1.8f + allowance[4], 0.0f + allowance[5]};
	fp32 pre_Au_reposition[6] = {0.708f + allowance[0], 0.72f + allowance[1], 0.5f + allowance[2], 0.0f + allowance[3], 1.8f + allowance[4], 0.0f + allowance[5]};
	fp32 Au_reposition[6] = {0.708f + allowance[0], 2.60f + allowance[1], 2.35f + allowance[2], 0.0f + allowance[3]  , 1.53f + allowance[4], 0.0f + allowance[5]};
	fp32 Ag_reposition[6] = {0.708f + allowance[0], 0.72f + allowance[1], 0.5f + allowance[2], 0.0f + allowance[3], 1.8f + allowance[4], 0.0f + allowance[5]};

	memcpy(arm_control.repostion_position, repositon_position, sizeof(repositon_position));
	memcpy(arm_control.gou_dong, gou_dong, sizeof(gou_dong));
	memcpy(arm_control.three_ore_position, three_ore_position, sizeof(three_ore_position));
	memcpy(arm_control.pre_Au_reposition, pre_Au_reposition, sizeof(pre_Au_reposition));
	memcpy(arm_control.Au_reposition, Au_reposition, sizeof(Au_reposition));
	memcpy(arm_control.Ag_reposition, Ag_reposition, sizeof(Ag_reposition));

	TD_init(&arm_control.arm_1_TD, 2.0f, 2.0f, 0.002f, arm_control.motor_YAW_data.DM_motor_measure->motor_position);
}

void arm_feedback_update(arm_control_t *arm_control_position, Robotic_6DOF_control_t *R_6D_ctrl, isomorphic_control_t* i_ctrl)
{

#if	SELF_CTRL_XYZYPR
	//xyzypr模式
	
	// 正解算输入
	R_6D_ctrl->Joint6D_FK.theta[0] = arm_control_position->motor_YAW_data.DM_motor_measure->motor_position;
	R_6D_ctrl->Joint6D_FK.theta[1] = -arm_message.target_position[1] - 0.279f;
	R_6D_ctrl->Joint6D_FK.theta[2] = arm_message.target_position[2] - 1.292f;
	R_6D_ctrl->Joint6D_FK.theta[3] = arm_message.target_position[3];
	R_6D_ctrl->Joint6D_FK.theta[4] = arm_message.target_position[4] - 1.80f;
	R_6D_ctrl->Joint6D_FK.theta[5] = -arm_message.target_position[5];
	// 正解算调用
	SolveFK(&R_6D_ctrl->Robotic_6D, &R_6D_ctrl->Joint6D_FK, &R_6D_ctrl->Pose6D_FK);

	if (Quaterniont_Mode)
	{
		float arm_pose_q_temp[4];
		float arm_pose_q_temp_2[4];
//		const float q0[4] = {0.707, 0, 0, 0.707}; // 绕z轴逆时针转90度
//		const float q0[4] = {1.0f, 0.0f, 0.0f, 0.0f};
		const float q0[4] = {0.707,0.0,0.0,0.707}; //绕z轴顺时针转90度
		const float q1[4] = {0.707,0.0,-0.707,0.0}; //绕x轴顺时针转90度
		float q_multiply_result[4];
		
		R_6D_ctrl->Pose6D_IK.X = arm_pose.x * 70.0f - 179.864f - 385.0f + sc_allowance[0];
		R_6D_ctrl->Pose6D_IK.Y = arm_pose.y * 80.0f;
		R_6D_ctrl->Pose6D_IK.Z = -arm_pose.z * 70.0f + 469.576f + 20.0f + sc_allowance[1];

		for (int i = 0; i < 4; i++)
		{
			arm_pose_q_temp[i] = arm_pose.q[i];
		}

		QuaternionMultiply(q0, arm_pose_q_temp, arm_pose_q_temp_2);
		QuaternionMultiply(q1, arm_pose_q_temp_2, q_multiply_result);

		R_6D_ctrl->Pose6D_IK.Q[0] = q_multiply_result[0];
		R_6D_ctrl->Pose6D_IK.Q[1] = q_multiply_result[1];
		R_6D_ctrl->Pose6D_IK.Q[2] = q_multiply_result[2];
		R_6D_ctrl->Pose6D_IK.Q[3] = q_multiply_result[3];
	}
#else 
		//同构模式
		for(uint8_t i=0;i<6;i++){
			i_ctrl->joint[i] = arm_position.joint[i];
		}
		if(arm_position.time_stamp != i_ctrl->last_stamp){
			i_ctrl->last_stamp = arm_position.time_stamp;
			i_ctrl->link_flag  = 1;
		}
		else{
			i_ctrl->link_flag = 0;
		}
#endif
}

void arm_control_key_check(all_key_t *arm_cnotrol_key_check)
{
	if (chassis.arm_mode == ONE_KEY_MODE)
	{
		key_itself_press_num(&(arm_cnotrol_key_check->capture_key), 2);
		key_itself_press_num(&(arm_cnotrol_key_check->exchange_key), 2);
	}
	else if(chassis.arm_mode == SELF_CONTROL_MODE)
	{
			key_itself_press_num(&(arm_cnotrol_key_check->self_control_rotate1_key),2);		
			key_itself_press_num(&(arm_cnotrol_key_check->self_control_rotate2_key),2);
	}
	key_itself_press_num(&(arm_cnotrol_key_check->suker_key), 2);
	key_itself_press_num(&(arm_cnotrol_key_check->ajx_on_key), 2);
}

void arm_control_set(arm_control_t *arm_control_set, all_key_t *arm_key)
{
	if ((arm_key->suker_key.itself.mode != arm_key->suker_key.itself.last_mode) || (last_s[4].itself.mode != last_s[4].itself.last_mode))
	{
		suker_key_flag = 1 - suker_key_flag;
	}
	if (suker_key_flag == 1)
	{
		HAL_GPIO_WritePin(Pump_GPIO_Port, Pump_Pin, GPIO_PIN_SET);
	}
	else if (suker_key_flag == 0)
	{
		HAL_GPIO_WritePin(Pump_GPIO_Port, Pump_Pin, GPIO_PIN_RESET);
	}
	
	if (arm_key->ajx_on_key.itself.mode != arm_key->ajx_on_key.itself.last_mode)
	{
		AJX_flag = 1 - AJX_flag;
	}

	if (chassis.chassis_mode == 0)
	{
		TD_set_x(&arm_control_set->arm_1_TD, arm_control_set->motor_YAW_data.DM_motor_measure->motor_position);
	}
	else
	{
		TD_calc(&arm_control_set->arm_1_TD, arm_control_set->motor_1_position);

		arm_control_set->motor_YAW_data.position_set = arm_control_set->arm_1_TD.x;
	}

	if (chassis.arm_mode == 1 && chassis.suker_key_flag == 0)
	{
		TD_set_r(&arm_control_set->arm_1_TD, 3.0f);
	}
	else if ((chassis.arm_mode == 0 || chassis.arm_mode == 2))
	{
		TD_set_r(&arm_control_set->arm_1_TD, 5.0f);
	}
	else
	{
		TD_set_r(&arm_control_set->arm_1_TD, 3.0f);
	}
	// 一键抓矿
	if (arm_key->capture_key.itself.mode != arm_key->capture_key.itself.last_mode)
	{
		arm_control_set->arm_position_flag = 0;
		if (arm_control_set->arm_move_flag == NORMAL_POSITION)
		{
			if (chassis.move_mode == Au)
			{
				if (HAL_GPIO_ReadPin(Ore_2_GPIO_Port, Ore_2_Pin) == 0)
				{
					arm_control_set->arm_move_flag = Au3;
				}
				else if (HAL_GPIO_ReadPin(Ore_1_GPIO_Port, Ore_1_Pin) == 0)
				{
					arm_control_set->arm_move_flag = Au2;
				}
				else
				{
					arm_control_set->arm_move_flag = Au1;
				}
			}
			else if (chassis.move_mode == Ag)
			{
				if (HAL_GPIO_ReadPin(Ore_2_GPIO_Port, Ore_2_Pin) == 0)
				{
					arm_control_set->arm_move_flag = Ag3;
				}
				else if (HAL_GPIO_ReadPin(Ore_1_GPIO_Port, Ore_1_Pin) == 0)
				{
					arm_control_set->arm_move_flag = Ag2;
				}
				else
				{
					arm_control_set->arm_move_flag = Ag1;
				}
			}
			else
			{
				if (HAL_GPIO_ReadPin(Ore_2_GPIO_Port, Ore_2_Pin) == 0)
				{
					arm_control_set->arm_move_flag = Ag3;
				}
				else if (HAL_GPIO_ReadPin(Ore_1_GPIO_Port, Ore_1_Pin) == 0)
				{
					arm_control_set->arm_move_flag = Ag2;
				}
				else
				{
					arm_control_set->arm_move_flag = Ag1;
				}
			}
		}
		else
		{
			arm_control_set->arm_move_flag = NORMAL_POSITION;
		}
		time_cnt = 0;
	}

	// 一键兑矿
		if (arm_key->exchange_key.itself.mode != arm_key->exchange_key.itself.last_mode)
		{
			arm_control_set->arm_position_flag = 0;
			if (arm_control_set->arm_move_flag == NORMAL_POSITION)
			{
				if (HAL_GPIO_ReadPin(Ore_2_GPIO_Port, Ore_2_Pin) == 0)
				{
					arm_control_set->arm_move_flag = EXCHANGE1;
				}
				else
				{
					arm_control_set->arm_move_flag = EXCHANGE2;
				}
			}
			else
			{
				arm_control_set->arm_move_flag = NORMAL_POSITION;
			}
			time_cnt = 0;
		}

	if (arm_control_set->arm_move_flag != NORMAL_POSITION && AJX_flag == 0)
	{
		arm_check_get_position(arm_control_set);
		if (arm_control_set->arm_get_position_flag > arm_control_set->arm_move_routine.flag_and_time[arm_control_set->arm_move_flag][arm_control_set->arm_position_flag][1])
		{
			arm_control_set->arm_position_flag++;
			arm_control_set->arm_get_position_flag = 0;
			if (arm_control_set->arm_move_routine.flag_and_time[arm_control_set->arm_move_flag][arm_control_set->arm_position_flag][0] == 0)
			{
				suker_key_flag = 0;
			}
			else if (arm_control_set->arm_move_routine.flag_and_time[arm_control_set->arm_move_flag][arm_control_set->arm_position_flag][0] == 1)
			{
				suker_key_flag = 1;
			}
		}

		if (arm_control_set->arm_position_flag == arm_control_set->routine_length[arm_control_set->arm_move_flag] - 1 && arm_control_set->arm_get_position_flag >= arm_control_set->arm_move_routine.flag_and_time[arm_control_set->arm_move_flag][arm_control_set->arm_position_flag][1])
		{
			// 连续一键
			if (arm_control_set->arm_move_flag == Ag1)
			{
				if (HAL_GPIO_ReadPin(Ore_1_GPIO_Port, Ore_1_Pin) == 0)
				{
					arm_control_set->arm_move_flag = Ag2;
					arm_control_set->arm_position_flag = 0;
				}
				else
				{
					arm_control_set->arm_move_flag = NORMAL_POSITION;
				}
			}
			else if (arm_control_set->arm_move_flag == Ag2)
			{
				if (HAL_GPIO_ReadPin(Ore_2_GPIO_Port, Ore_2_Pin) == 0)
				{
					arm_control_set->arm_move_flag = NORMAL_POSITION;
					arm_control_set->arm_position_flag = 0;
				}
				else
				{
					arm_control_set->arm_move_flag = NORMAL_POSITION;
				}
			}
			else if(arm_control_set->arm_move_flag == Ag3)
			{
					arm_control_set->arm_move_flag = NORMAL_POSITION;
					arm_control_set->arm_position_flag = 0;
			}
			else if (arm_control_set->arm_move_flag == EXCHANGE1)
			{
				arm_control_set->arm_position_flag = 0;
				arm_control_set->arm_move_flag = NORMAL_POSITION;
			}
			else if (arm_control_set->arm_move_flag == EXCHANGE2)
			{
				arm_control_set->arm_move_flag = NORMAL_POSITION;
				arm_control_set->arm_position_flag = 0;
			}
			else if (arm_control_set->arm_move_flag == Au1)
			{
				if (HAL_GPIO_ReadPin(Ore_1_GPIO_Port, Ore_1_Pin) == 0)
				{
					arm_control_set->arm_move_flag = Au2;
					arm_control_set->arm_position_flag = 0;
				}
				else
				{
					arm_control_set->arm_move_flag = NORMAL_POSITION;
				}
			}
			else if (arm_control_set->arm_move_flag == Au2)
			{
				if (HAL_GPIO_ReadPin(Ore_2_GPIO_Port, Ore_2_Pin) == 0)
				{
					arm_control_set->arm_move_flag = Au3;
					arm_control_set->arm_position_flag = 0;
				}
				else
				{
					arm_control_set->arm_move_flag = NORMAL_POSITION;
				}
			}
			else if (arm_control_set->arm_move_flag == Au3)
			{
				arm_control_set->arm_move_flag = NORMAL_POSITION;
			}
		}
	}
	else
	{
		arm_control_set->arm_get_position_flag = 0;
	}
}

float AbsMaxOf6(Joint6D_t _joints)
{
	float max = -1;
	for (uint8_t i = 0; i < 5; i++)
	{
		if (fabs(_joints.theta[i]) > max)
		{
			max = fabs(_joints.theta[i]);
		}
	}

	return max;
}

float AbsMaxWithJoint4Priority(Joint6D_t _joints)
{
	float weights[] = {1.0, 1.0, 1.0, 0.2, 1.0, 1.0}; // joint4的权重较低
	float max = 0;
	for (uint8_t i = 0; i < 6; i++)
	{
		float weighted = fabs(_joints.theta[i]) * weights[i];
		if (weighted > max)
		{
			max = weighted;
		}
	}
	return max;
}

Joint6D_t deltaJoints;

void MoveL(Robotic_6DOF_control_t *R_6D_ctrl, arm_control_t *arm_control_set, all_key_t *arm_key)
{
	Joint6D_t tmp;
	bool valid[8];
	bool SolveIK_Success;
//	bool new_joint_flag = false;
	// 机械臂逆解算
	for(int i = 0; i < 6;i++)
	{
		R_6D_ctrl->Last_Joint_Final[i] = R_6D_ctrl->Joint_Final[i];
	}
	
	SolveIK_Success = SolveIK(&R_6D_ctrl->Robotic_6D, &R_6D_ctrl->Pose6D_IK,
							  &R_6D_ctrl->output_solvers_IK, &Last_Joint6D, Quaterniont_Mode);

	if (!SolveIK_Success)
	{
		return;
	}
	else
	{
		R_6D_ctrl->ValidCnt = 0;
		for (int i = 0; i < 8; i++)
		{
			valid[i] = true;

			for (int j = 0; j < 6; j++)
			{
				//								if (j == 1)
				//								{
				//									if (-(R_6D_ctrl->output_solvers_IK.theta[i][j]+0.279) > R_6D_ctrl->Joint[j].angleLimitMax ||
				//											-(R_6D_ctrl->output_solvers_IK.theta[i][j]+0.279) < R_6D_ctrl->Joint[j].angleLimitMin)
				//									{
				//											valid[i] = false;
				//											continue;
				//									}
				//								}
				//								if (j == 2)
				//								{
				//									if (R_6D_ctrl->output_solvers_IK.theta[i][j]+1.292 > R_6D_ctrl->Joint[j].angleLimitMax ||
				//											R_6D_ctrl->output_solvers_IK.theta[i][j]+1.292 < R_6D_ctrl->Joint[j].angleLimitMin)
				//									{
				//											valid[i] = false;
				//											continue;
				//									}
				//								}
				//								else
				//								{
				if (R_6D_ctrl->output_solvers_IK.theta[i][j] > R_6D_ctrl->Joint[j].angleLimitMax ||
					R_6D_ctrl->output_solvers_IK.theta[i][j] < R_6D_ctrl->Joint[j].angleLimitMin)
				{
					valid[i] = false;
					continue;
				}
				//								}
			}

			if (valid[i])
				R_6D_ctrl->ValidCnt++;
		}

		if (R_6D_ctrl->ValidCnt)
		{
			float min = 1000.0f;
			uint8_t indexConfig = 0;
			for (int i = 0; i < 8; i++)
			{
				if (valid[i])
				{
//					if(fabs(R_6D_ctrl->output_solvers_IK.theta[i][0] - Last_Joint6D.theta[0]) > fabs(R_6D_ctrl->output_solvers_IK.theta[i][0] - Last_Joint6D.theta[0]))
					for (int j = 0; j < 6; j++)
					{
						R_6D_ctrl->Joint_Valid[i][j] = R_6D_ctrl->output_solvers_IK.theta[i][j];
						Last_Joint6D.theta[j] = R_6D_ctrl->output_solvers_IK.theta[i][j];
						tmp.theta[j] = R_6D_ctrl->Joint6D_FK.theta[j] - R_6D_ctrl->output_solvers_IK.theta[i][j];
					}
					float maxAngle = AbsMaxWithJoint4Priority(tmp);
					if (maxAngle < min)
					{
						min = maxAngle;
						indexConfig = i;
					}
				}
				else
				{
					for (int j = 0; j < 6; j++)
					{
						R_6D_ctrl->Joint_Valid[i][j] = 0.0f;
					}
				}
			}
			R_6D_ctrl->Joint_Final[0] = R_6D_ctrl->output_solvers_IK.theta[indexConfig][0];
			R_6D_ctrl->Joint_Final[1] = R_6D_ctrl->output_solvers_IK.theta[indexConfig][1];
			R_6D_ctrl->Joint_Final[2] = R_6D_ctrl->output_solvers_IK.theta[indexConfig][2];
			R_6D_ctrl->Joint_Final[3] = R_6D_ctrl->output_solvers_IK.theta[indexConfig][3];
			R_6D_ctrl->Joint_Final[4] = R_6D_ctrl->output_solvers_IK.theta[indexConfig][4];
			R_6D_ctrl->Joint_Final[5] = R_6D_ctrl->output_solvers_IK.theta[indexConfig][5];

			arm_target_position[0] = (R_6D_ctrl->output_solvers_IK.theta[indexConfig][0] + 0.564f);
			arm_target_position[1] = -(R_6D_ctrl->output_solvers_IK.theta[indexConfig][1] + 0.279f);
			arm_target_position[2] = R_6D_ctrl->output_solvers_IK.theta[indexConfig][2] + 1.292f;
			arm_target_position[3] = R_6D_ctrl->output_solvers_IK.theta[indexConfig][3];
			arm_target_position[4] = (R_6D_ctrl->output_solvers_IK.theta[indexConfig][4] + 1.80f);
			arm_target_position[5] = -(R_6D_ctrl->output_solvers_IK.theta[indexConfig][5]);
		}
		else
		{
			for (int j = 0; j < 6; j++)
			{
				R_6D_ctrl->Joint_Final[j] = 0.0f;
			}
		}
	}
	return;
}

int l = 0;

void arm_control_loop(Robotic_6DOF_control_t *R_6D_ctrl, arm_control_t *arm_control_loop, all_key_t *arm_key, isomorphic_control_t* i_ctrl)
{
	if (chassis.arm_mode == ONE_KEY_MODE )
	{
		if (arm_control_loop->arm_move_flag == NORMAL_POSITION)
		{
			switch (chassis.move_mode)
			{
					case Home:
					{
						for (int i = 0; i < 6; i++)
						{
							arm_control_loop->one_key_position[i] = arm_control_loop->repostion_position[i];
						}
						break;
					}
					case Ag:
					{
						for (int i = 0; i < 6; i++)
						{
							arm_control_loop->one_key_position[i] = arm_control_loop->Ag_reposition[i];
						}
						break;
					}
					case Au:
					{
						for (int i = 0; i < 6; i++)
						{
								arm_control_loop->one_key_position[i] = arm_control_loop->Au_reposition[i];
						}
						break;
					}
					case Exchange:
					{
						for (int i = 0; i < 6; i++)
						{
							arm_control_loop->one_key_position[i] = arm_control_loop->repostion_position[i];
						}
						break;
					}
					case GouDong:
					{
						for (int i = 0; i < 6; i++)
						{
							arm_control_loop->one_key_position[i] = arm_control_loop->gou_dong[i];
						}
						break;
					}
					case Wait:
					{
						if(chassis.last_move_mode == Au)
						{
							for(int i = 0; i < 6; i++)
							{
									arm_control_loop->one_key_position[i] = arm_control_loop->repostion_position[i];
							}
							if(fabs(arm_control_loop->one_key_position[0] - arm_control_loop->motor_YAW_data.position_set) < 0.3f &&
									fabs(arm_control_loop->one_key_position[1] - (arm_message.target_position[1])) < 0.1f &&
									fabs(arm_control_loop->one_key_position[2] - (arm_message.target_position[2])) < 0.1f &&
									fabs(arm_control_loop->one_key_position[3] - (arm_message.target_position[3])) < 0.2f &&
									fabs(arm_control_loop->one_key_position[4] - (arm_message.target_position[4])) < 0.15f &&
									fabs(arm_control_loop->one_key_position[5] - (arm_message.target_position[5])) < 0.15f)
							{
									chassis.move_mode = Home;
									chassis.yaw_angle_set = rad_format(chassis.yaw_angle_set -= 3 * PI / 4);
									chassis.last_move_mode = chassis.move_mode;
							}
						}
						else if(chassis.last_move_mode == Home)
						{
							fp32 init_position[6] = {0.708f + allowance[0], 0.8f + allowance[1], 0.5f + allowance[2], 0.0f + allowance[3], 1.8f + allowance[4], 0.0f + allowance[5]};
							for(int i = 0; i < 6; i++)
							{
									arm_control_loop->one_key_position[i] = init_position[i];
							}
							if(time_cnt < 2000)
							{
								time_cnt ++;
							}
							else
							{
								time_cnt = 0;
								chassis.move_mode = Home;
								chassis.last_move_mode = chassis.move_mode;
							}
						}
					}
					default:
					{
						break;
					}
			}
		}
		else if (arm_control_loop->arm_move_flag != NORMAL_POSITION)
		{
			if(AJX_flag == 0)
			{
				for (int i = 0; i < 6; i++)
				{
					if (arm_control_loop->arm_move_flag == Ag1)
					{
						arm_control_loop->one_key_position[i] = arm_control_loop->arm_move_routine.Ag1[arm_control_loop->arm_position_flag][i];
					}
					else if (arm_control_loop->arm_move_flag == Ag2)
					{
						arm_control_loop->one_key_position[i] = arm_control_loop->arm_move_routine.Ag2[arm_control_loop->arm_position_flag][i];
					}
					else if (arm_control_loop->arm_move_flag == Ag3)
					{
						arm_control_loop->one_key_position[i] = arm_control_loop->arm_move_routine.Ag3[arm_control_loop->arm_position_flag][i];
					}
					else if (arm_control_loop->arm_move_flag == EXCHANGE1)
					{
						arm_control_loop->one_key_position[i] = arm_control_loop->arm_move_routine.exchange1[arm_control_loop->arm_position_flag][i];
					}
					else if (arm_control_loop->arm_move_flag == EXCHANGE2)
					{
						arm_control_loop->one_key_position[i] = arm_control_loop->arm_move_routine.exchange2[arm_control_loop->arm_position_flag][i];
					}
					else if (arm_control_loop->arm_move_flag == Au1)
					{
						arm_control_loop->one_key_position[i] = arm_control_loop->arm_move_routine.Au1[arm_control_loop->arm_position_flag][i];
					}
					else if (arm_control_loop->arm_move_flag == Au2)
					{
						arm_control_loop->one_key_position[i] = arm_control_loop->arm_move_routine.Au2[arm_control_loop->arm_position_flag][i];
					}
					else if (arm_control_loop->arm_move_flag == Au3)
					{
						arm_control_loop->one_key_position[i] = arm_control_loop->arm_move_routine.Au3[arm_control_loop->arm_position_flag][i];
					}
				}
				if(arm_control_loop->arm_move_flag == EXCHANGE2)
				{
					if(arm_control_loop->arm_position_flag == 1)
					{
						clamp_flag = 1;
					}
					else if(arm_control_loop->arm_position_flag == 2)
					{
						time_cnt++;
						if(time_cnt > 400)
						{
							clamp_flag = 0;
						}
					}
					else if(arm_control_loop->arm_position_flag == 3)
					{
						time_cnt = 0;
						clamp_flag = 1;
					}
				}
				else if(arm_control_loop->arm_move_flag == EXCHANGE1)
				{
					if(arm_control_loop->arm_position_flag == 1)
					{
						clamp_flag = 1;
					}
					else if(arm_control_loop->arm_position_flag == 2)
					{
						time_cnt ++;
						if(time_cnt > 100)
						{
							clamp_flag = 0;
						}
					}
					else if(arm_control_loop->arm_position_flag == 3)
					{
						time_cnt = 0;
						clamp_flag = 1;
					}
				}
//				else if(arm_control_loop->arm_move_flag == Ag1 || arm_control_loop->arm_move_flag == Ag2 ||
//									arm_control_loop->arm_move_flag == Au1 || arm_control_loop->arm_move_flag == Au2)
//				{
//					if(arm_control_loop->arm_position_flag == 4)
//					{
//						time_cnt = 0;
//						clamp_flag = 1;
//					}
//					else if(arm_control_loop->arm_position_flag == 9)
//					{
//						clamp_flag = 0;
//						time_cnt ++;
//						if(time_cnt > 50)
//						{
//							clamp_flag = 1;
//						}
//						else if(time_cnt > 450)
//						{
//							clamp_flag = 0;
//						}
//					}
//					else if(arm_control_loop->arm_position_flag ==10)
//					{
//						time_cnt = 0;
//					}
//				}
			}
			else
			{
				if(time_cnt % 5 == 0)
				{
					arm_control_loop->one_key_position[0] = (pc_receive_msg.rx_data.motor1_position + nx_allowance[0]);
					arm_control_loop->one_key_position[1] = (pc_receive_msg.rx_data.motor2_position - nx_allowance[1]);
					arm_control_loop->one_key_position[2] = -(pc_receive_msg.rx_data.motor3_position - nx_allowance[2]);
					arm_control_loop->one_key_position[3] = (pc_receive_msg.rx_data.motor4_position - nx_allowance[3]);
					arm_control_loop->one_key_position[4] = -(pc_receive_msg.rx_data.motor5_position + nx_allowance[4]);
					arm_control_loop->one_key_position[5] = (pc_receive_msg.rx_data.motor6_position - nx_allowance[5]);
				}
					time_cnt++;
					if(time_cnt > time_flag[arm_control_loop->arm_move_flag][0][1] && time_cnt < time_flag[arm_control_loop->arm_move_flag][1][1])
					{
							suker_key_flag = time_flag[arm_control_loop->arm_move_flag][0][0];
					}
					else if(time_cnt > time_flag[arm_control_loop->arm_move_flag][1][1])
					{
							suker_key_flag = time_flag[arm_control_loop->arm_move_flag][1][0];
							if (fabs(arm_control_loop->motor_1_position - 0.564f) < 0.3f &&
							fabs(arm_control_loop->motor_2_position - 0.0f) < 0.1f &&
							fabs(arm_control_loop->motor_3_position - 0.0f) < 0.1f &&
							fabs(arm_control_loop->motor_4_position - 0.0f) < 0.2f &&
							fabs(arm_control_loop->motor_5_position - 1.8f) < 0.15f &&
							fabs(arm_control_loop->motor_6_position - 0.0f) < 0.15f)
							{
								arm_control_loop->arm_move_flag = NORMAL_POSITION;
								time_cnt = 0;
							}
					}
			}
		}
			arm_target_position[0] = arm_control_loop->one_key_position[0];
			arm_target_position[1] = arm_control_loop->one_key_position[1];
			arm_target_position[2] = arm_control_loop->one_key_position[2];
			arm_target_position[3] = arm_control_loop->one_key_position[3];
			arm_target_position[4] = arm_control_loop->one_key_position[4];
			arm_target_position[5] = arm_control_loop->one_key_position[5];
	}
	else if (chassis.arm_mode == SELF_CONTROL_MODE)
	{
#if		SELF_CTRL_XYZYPR
		//					if(chassis.last_chassis_mode != SELF_CONTROL_MODE)
		//					{
		//							R_6D_ctrl->Pose6D_IK_Z_TD.x = 0.0f;
		//					}
		//					TD_calc(&R_6D_ctrl->Pose6D_IK_Z_TD,R_6D_ctrl->Pose6D_IK.Z);
		//					R_6D_ctrl->Pose6D_IK.Z = R_6D_ctrl->Pose6D_IK_Z_TD.x;
		MoveL(R_6D_ctrl, arm_control_loop, arm_key);
		if(arm_key->self_control_rotate1_key.itself.mode != arm_key->self_control_rotate1_key.itself.last_mode)
		{
				Half_rod_cnt ++;
		}
		else if(arm_key->self_control_rotate2_key.itself.mode != arm_key->self_control_rotate2_key.itself.last_mode)
		{
				Half_rod_cnt --;
		}
		arm_target_position[5] = rad_format(arm_target_position[5] + PI / 2 * Half_rod_cnt);
#else 
		//同构模式
			if(chassis.arm_mode != SELF_CONTROL_MODE)
			{
					i_ctrl->joint5_offset = i_ctrl->joint[5];
			}
			if(i_ctrl->link_flag == 1){
				for(uint8_t j=0;j<6;j++){
					arm_target_position[j] = i_ctrl->joint[j];
				}
				arm_target_position[5] -= i_ctrl->joint5_offset;
				if(arm_key->self_control_rotate1_key.itself.mode != arm_key->self_control_rotate1_key.itself.last_mode)
				{
						Half_rod_cnt ++;
				}
				else if(arm_key->self_control_rotate2_key.itself.mode != arm_key->self_control_rotate2_key.itself.last_mode)
				{
						Half_rod_cnt --;
				}
				arm_target_position[5] = rad_format(arm_target_position[5] + PI / 2 * Half_rod_cnt);
			}
#endif
	}
	else if (chassis.arm_mode == NX_CONTROL_MODE)
	{
		arm_target_position[0] = (pc_receive_msg.rx_data.motor1_position + nx_allowance[0]);
		arm_target_position[1] = (pc_receive_msg.rx_data.motor2_position - nx_allowance[1]);
		arm_target_position[2] = -(pc_receive_msg.rx_data.motor3_position - nx_allowance[2]);
		arm_target_position[3] = (pc_receive_msg.rx_data.motor4_position - nx_allowance[3]);
		arm_target_position[4] = -(pc_receive_msg.rx_data.motor5_position + nx_allowance[4]);
		arm_target_position[5] = (pc_receive_msg.rx_data.motor6_position - nx_allowance[5]);
	}

	arm_control_loop->motor_1_position = arm_target_position[0];
	arm_control_loop->motor_2_position = arm_target_position[1];
	arm_control_loop->motor_3_position = arm_target_position[2];
	arm_control_loop->motor_4_position = arm_target_position[3];
	arm_control_loop->motor_5_position = arm_target_position[4];
	arm_control_loop->motor_6_position = arm_target_position[5];
}

void arm_check_get_position(arm_control_t *check_position)
{
	if (fabs(check_position->motor_1_position - check_position->motor_YAW_data.position_set) < 0.3f &&
		fabs(check_position->motor_2_position - (arm_message.target_position[1])) < 0.1f &&
		fabs(check_position->motor_3_position - (arm_message.target_position[2])) < 0.1f &&
		fabs(check_position->motor_4_position - (arm_message.target_position[3])) < 0.2f &&
		fabs(check_position->motor_5_position - (arm_message.target_position[4])) < 0.15f &&
		fabs(check_position->motor_6_position - (arm_message.target_position[5])) < 0.15f)
	{
		check_position->arm_get_position_flag++;
	}
	else
	{
		check_position->arm_get_position_flag = 0;
	}
}
