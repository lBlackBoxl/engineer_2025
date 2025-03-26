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

static void arm_control_init(all_key_t *arm_control_key_init, Robotic_6DOF_control_t *R_6D_ctrl);
static void arm_feedback_update( arm_control_t *arm_control_position, Robotic_6DOF_control_t *R_6D_ctrl);
static void arm_control_set( arm_control_t *arm_control_set, all_key_t * arm_key);
static void arm_check_get_position(arm_control_t *check_position);
static void arm_control_loop(Robotic_6DOF_control_t *R_6D_ctrl,arm_control_t *arm_control_loop, all_key_t *arm_key); 

arm_control_t arm_control;
Robotic_6DOF_control_t R_6D_ctrl;
Ag_Catch_t Ag_Catch;

float max_joint_speed[6];
Joint6D_t Last_Joint6D;
uint8_t suker_key_flag;
int G_flag;
float32_t yaw_angle_set;

void arm_control_task(void const * argument)
{
		arm_control_init(&all_key,&R_6D_ctrl);
		Joint6D_Init(&R_6D_ctrl.Robotic_6D);
		
		uint32_t system_clock = osKernelSysTick();
		while(1)
		{
				arm_control.dt = DWT_GetDeltaT(&arm_control.DWT_Count);
				
				arm_feedback_update(&arm_control,&R_6D_ctrl);
				arm_control_set(&arm_control, &all_key);
				arm_control_loop(&R_6D_ctrl, &arm_control,&all_key);
				osDelay(2);
				CAN_cmd_4310_mit(arm_control.motor_YAW_data.position_set,0.0f,800.0f,10.0f,0.0f,DM_YAW_TX_ID,hcan2);
				DWT_Delay(0.0003f);
		}
}

void arm_control_init(all_key_t *arm_control_key_init, Robotic_6DOF_control_t *R_6D_ctrl)
{
		//YAW关节电机初始化
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
		
		//各关节角度限制
		R_6D_ctrl->Joint[0].angleLimitMax =  (175.0f/180.0f)*PI - PI/2.0f;
		R_6D_ctrl->Joint[0].angleLimitMin = -(175/180)*PI - PI/2.0f;
		
		R_6D_ctrl->Joint[1].angleLimitMax =  (140.0f/180.0f)*PI;
		R_6D_ctrl->Joint[1].angleLimitMin =  0.0f;
		
		R_6D_ctrl->Joint[2].angleLimitMax =  (120.0f/180.0f)*PI;
		R_6D_ctrl->Joint[2].angleLimitMin =  0;
		
		R_6D_ctrl->Joint[3].angleLimitMax =  (PI/2.0f + 0.10f*PI);//100-110度;
		R_6D_ctrl->Joint[3].angleLimitMin = -(PI/2.0f + 0.10f*PI);//-100-110度;
		
		R_6D_ctrl->Joint[4].angleLimitMax =  PI;
		R_6D_ctrl->Joint[4].angleLimitMin = -PI;
		
		for(int i = 0; i < 5; i++)
		{
				R_6D_ctrl->Joint_Final[i] = 0.0f;
		}
//		uint8_t routine_length[9] = {12, 12, 10, 6, 7, 10, 10 ,3};
		uint8_t routine_length[9] = {10, 12, 10, 6, 6, 10, 10 ,3};
		
		for(int i = 0; i < 9; i++)
		{
				arm_control.routine_length[i] = routine_length[i];
		}
		TD_init(&R_6D_ctrl->Pose6D_IK_Z_TD, 50.0f, 2.0f, 0.002f, R_6D_ctrl->Pose6D_IK.Z);
		
		//机械臂复位位置
		fp32 allowance[6] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
		fp32 repositon_position[6] = {0.0f + allowance[0],0.0f + allowance[1],0.0f + allowance[2],0.0f + allowance[3],2.31f + allowance[4],0.0f + allowance[5]};		
		fp32 three_ore_position[6] = {0.0f + allowance[0],0.0f + allowance[1],0.0f + allowance[2],0.0f + allowance[3],0.0f + allowance[4],0.0f + allowance[5]};	
		fp32 pre_Au_reposition[6] = {0.0f + allowance[0],0.0f + allowance[1],0.0f + allowance[2],0.0f + allowance[3],0.0f + allowance[4],0.0f + allowance[5]};
		fp32 Au_reposition[6] = {0.0f + allowance[0],0.0f + allowance[1],0.0f + allowance[2],0.0f + allowance[3],0.0f + allowance[4],0.0f + allowance[5]};
		fp32 Ag_reposition[6] = {0.0f + allowance[0],0.0f + allowance[1],0.0f + allowance[2],0.0f + allowance[3],0.0f + allowance[4],0.0f + allowance[5]};		
		memcpy(arm_control.repostion_position,repositon_position,sizeof(repositon_position));
		memcpy(arm_control.three_ore_position,three_ore_position,sizeof(three_ore_position));
		memcpy(arm_control.pre_Au_reposition,pre_Au_reposition,sizeof(pre_Au_reposition));
		memcpy(arm_control.Au_reposition,Au_reposition,sizeof(Au_reposition));
		memcpy(arm_control.Ag_reposition,Ag_reposition,sizeof(Ag_reposition));
}

void arm_feedback_update( arm_control_t *arm_control_position, Robotic_6DOF_control_t *R_6D_ctrl)
{
	 if(Quaterniont_Mode)
	 {
				R_6D_ctrl->Pose6D_IK.X = arm_pose.x;
				R_6D_ctrl->Pose6D_IK.Y = arm_pose.y;
				R_6D_ctrl->Pose6D_IK.Z = arm_pose.z;
				R_6D_ctrl->Pose6D_IK.Q[0] = arm_pose.q[0];
				R_6D_ctrl->Pose6D_IK.Q[1] = arm_pose.q[1];
				R_6D_ctrl->Pose6D_IK.Q[2] = arm_pose.q[2];
				R_6D_ctrl->Pose6D_IK.Q[3] = arm_pose.q[3];	 
	 }
	 if(chassis.chassis_mode == SELF_CONTROL_MODE && chassis.last_chassis_mode!=SELF_CONTROL_MODE)
	 {
				yaw_angle_set = arm_message.target_position[5];
	 }
}

void arm_control_set(arm_control_t *arm_control_set, all_key_t *arm_key)
{
		
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

Joint6D_t deltaJoints;
bool xyz_valid[4];
bool valid[4];

void MoveL(Robotic_6DOF_control_t *R_6D_ctrl, arm_control_t *arm_control_set, all_key_t *arm_key)
{
		bool SolveIK_Success;
		//机械臂逆解算
		SolveIK_Success = SolveIK(&R_6D_ctrl->Robotic_6D, &R_6D_ctrl->Pose6D_IK,
															&R_6D_ctrl->output_solvers_IK, &Last_Joint6D, Quaterniont_Mode);

		if(!SolveIK_Success)
		{
				return;
		}		
		else
		{
				R_6D_ctrl->xyz_ValidCnt = 0;
				R_6D_ctrl->ValidCnt = 0;			
				for (int i = 0; i < 4; i++)
				{
						xyz_valid[i] = true;
						valid[i] = true;
							
						for (int j = 0; j < 2; j++)
						{
								if (R_6D_ctrl->output_solvers_IK.theta[i][j] > R_6D_ctrl->Joint[j].angleLimitMax ||
										R_6D_ctrl->output_solvers_IK.theta[i][j] < R_6D_ctrl->Joint[j].angleLimitMin)
								{
										xyz_valid[i] = false;
								}
								
						}							
						for (int j = 0; j < 5; j++)
						{
								if (R_6D_ctrl->output_solvers_IK.theta[i][j] > R_6D_ctrl->Joint[j].angleLimitMax ||
										R_6D_ctrl->output_solvers_IK.theta[i][j] < R_6D_ctrl->Joint[j].angleLimitMin)
								{
										valid[i] = false;
								}
								
						}	
							if (xyz_valid[i]) R_6D_ctrl->xyz_ValidCnt++;
							if (valid[i]) R_6D_ctrl->ValidCnt++;
				}                                                                                                 

				if (R_6D_ctrl->xyz_ValidCnt && (!R_6D_ctrl->ValidCnt))
				{
						float go_delta_min = 100.0f;
						float go_max_position = 0.0f;
						uint8_t indexConfig = 0;
						for (int i = 0; i < 4; i++)
						{

								if (xyz_valid[i])
								{
										if(fabs(R_6D_ctrl->Pose6D_IK.Y* motor_to_real * sc_to_arm )<200)
										{
												if(fabs(R_6D_ctrl->output_solvers_IK.theta[i][0] - Last_Joint6D.theta[0]) < go_delta_min)
												{
														go_delta_min = fabs(R_6D_ctrl->output_solvers_IK.theta[i][0] - Last_Joint6D.theta[0]);
														indexConfig = i;
												}
										}
										else
										{
												if(R_6D_ctrl->Pose6D_IK.Y < 0)
												{
														if (R_6D_ctrl->output_solvers_IK.theta[i][0] >= go_max_position)
														{
																go_max_position = R_6D_ctrl->output_solvers_IK.theta[i][0]; 
																indexConfig = i;
														}
												}
												else
												{
														if (R_6D_ctrl->output_solvers_IK.theta[i][0] <= go_max_position)
														{
																go_max_position = R_6D_ctrl->output_solvers_IK.theta[i][0]; 
																indexConfig = i;
														}									
												}
										}
								}
						}
						R_6D_ctrl->Joint_Final[0] 	= 	R_6D_ctrl->output_solvers_IK.theta[indexConfig][0];
						R_6D_ctrl->Joint_Final[1]   =   R_6D_ctrl->output_solvers_IK.theta[indexConfig][1];			
						for (int j = 0; j < 2; j++)
						{
								Last_Joint6D.theta[j] = R_6D_ctrl->Joint_Final[j];
						}					
				}
				else if(R_6D_ctrl->ValidCnt)
				{		
					  float go_delta_min = 100.0f;
						float go_max_position = 0.0f;
						uint8_t indexConfig = 0;
						for (int i = 0; i < 4; i++)
						{			
								if (valid[i])
								{
										if(fabs(R_6D_ctrl->Pose6D_IK.Y* motor_to_real * sc_to_arm)<200)
										{
											if(fabs(R_6D_ctrl->output_solvers_IK.theta[i][0] - Last_Joint6D.theta[0]) < go_delta_min)
											{
												go_delta_min = fabs(R_6D_ctrl->output_solvers_IK.theta[i][0] - Last_Joint6D.theta[0]);
												indexConfig = i;
											}
										}
										else
										{
												if(R_6D_ctrl->Pose6D_IK.Y < 0)
												{
														if (R_6D_ctrl->output_solvers_IK.theta[i][0] >= go_max_position)
														{
																go_max_position = R_6D_ctrl->output_solvers_IK.theta[i][0]; 
																indexConfig = i;
														}
												}
												else
												{
														if (R_6D_ctrl->output_solvers_IK.theta[i][0] <= go_max_position)
														{
																go_max_position = R_6D_ctrl->output_solvers_IK.theta[i][0]; 
																indexConfig = i;
														}									
												}
										}
								}
						}	
						//3508向上为负
						R_6D_ctrl->Joint_Final[0] 	= 	R_6D_ctrl->output_solvers_IK.theta[indexConfig][0];
						R_6D_ctrl->Joint_Final[1]   =   R_6D_ctrl->output_solvers_IK.theta[indexConfig][1];
						R_6D_ctrl->Joint_Final[2]   =   R_6D_ctrl->output_solvers_IK.theta[indexConfig][2];
						R_6D_ctrl->Joint_Final[3]   =   -R_6D_ctrl->output_solvers_IK.theta[indexConfig][3];			
						for (int j = 0; j < 5; j++)
						{
								Last_Joint6D.theta[j] = R_6D_ctrl->Joint_Final[j];
						}
			}
			if(R_6D_ctrl->xyz_ValidCnt)
			{						
						//motor2 first
						if(fabs(40.0f - arm_message.target_position[0]) > 0.3f)
						{
								arm_control_set->motor_1_position       =  	 14.9f;
								arm_control_set->motor_2_position       =    40.0f;
								arm_control_set->motor_3_position       =    0.0f;
								arm_control_set->motor_4_position			  =	   0.0f;
								arm_control_set->motor_5_position 	  	=		 -1.57f;
								arm_control_set->motor_6_position 			=	   2.13f;						
						}
						else
						{
								arm_control_set->motor_1_position       =  	 R_6D_ctrl->high + 14.9f;
								arm_control_set->motor_2_position       =    40.0f;
								arm_control_set->motor_3_position       =    R_6D_ctrl->Joint_Final[0] ;
								arm_control_set->motor_4_position			  =	   R_6D_ctrl->Joint_Final[1] ;
								arm_control_set->motor_5_position 	  	=		 R_6D_ctrl->Joint_Final[2] - 1.57f;
								arm_control_set->motor_6_position 			=	   -R_6D_ctrl->Joint_Final[3] + 2.58f - 0.56f;//2006需要复位导致的零点变化				
						}
				}
		}
		return;
}

void arm_control_loop(Robotic_6DOF_control_t *R_6D_ctrl,arm_control_t *arm_control_loop, all_key_t *arm_key)
{
		if(chassis.arm_mode == ONE_KEY_MODE)
		{
				for(int i = 0; i < 6; i++)
				{
						if(arm_control_loop->arm_move_flag == Ag1)
						{
							arm_control_loop->one_key_position[i] = arm_control_loop->arm_move_routine.Ag1[arm_control_loop->arm_position_flag][i];
						}
						else if(arm_control_loop->arm_move_flag == Ag2)
						{
							arm_control_loop->one_key_position[i] = arm_control_loop->arm_move_routine.Ag2[arm_control_loop->arm_position_flag][i];
						}
						else if(arm_control_loop->arm_move_flag == Ag3)
						{
							arm_control_loop->one_key_position[i] = arm_control_loop->arm_move_routine.Ag3[arm_control_loop->arm_position_flag][i];
						}
						else if(arm_control_loop->arm_move_flag == EXCHANGE1)
						{
							arm_control_loop->one_key_position[i] = arm_control_loop->arm_move_routine.exchange1[arm_control_loop->arm_position_flag][i];
						}
						else if(arm_control_loop->arm_move_flag == EXCHANGE2)
						{
							arm_control_loop->one_key_position[i] = arm_control_loop->arm_move_routine.exchange2[arm_control_loop->arm_position_flag][i];
						}
						else if(arm_control_loop->arm_move_flag == Au1)
						{
							arm_control_loop->one_key_position[i] = arm_control_loop->arm_move_routine.Au1[arm_control_loop->arm_position_flag][i];
						}
						else if(arm_control_loop->arm_move_flag == Au2)
						{
							arm_control_loop->one_key_position[i] = arm_control_loop->arm_move_routine.Au2[arm_control_loop->arm_position_flag][i];
						}
						else if(arm_control_loop->arm_move_flag == Au3)
						{
							arm_control_loop->one_key_position[i] = arm_control_loop->arm_move_routine.Au3[arm_control_loop->arm_position_flag][i];
						}
				}
			
				if(arm_control_loop->arm_move_flag == NORMAL_POSITION)
				{
						arm_control_loop->one_key_position[0] = arm_control_loop->repostion_position[0];
						arm_control_loop->one_key_position[1] = arm_control_loop->repostion_position[1];
						arm_control_loop->one_key_position[2] = arm_control_loop->repostion_position[2];
						arm_control_loop->one_key_position[3] = arm_control_loop->repostion_position[3];
						arm_control_loop->one_key_position[4] = arm_control_loop->repostion_position[4];
						arm_control_loop->one_key_position[5] = arm_control_loop->repostion_position[5];
				}
				
				arm_control_loop->motor_1_position = arm_control_loop->one_key_position[0];
				arm_control_loop->motor_2_position = arm_control_loop->one_key_position[1];
				arm_control_loop->motor_3_position = arm_control_loop->one_key_position[2];
				arm_control_loop->motor_4_position = arm_control_loop->one_key_position[3];
				arm_control_loop->motor_5_position = arm_control_loop->one_key_position[4];
				arm_control_loop->motor_6_position = arm_control_loop->one_key_position[5];
		}
		else if(chassis.arm_mode == SELF_CONTROL_MODE)
		{
					if(chassis.last_chassis_mode != SELF_CONTROL_MODE)
					{
							R_6D_ctrl->Pose6D_IK_Z_TD.x = 0.0f;
					}
					TD_calc(&R_6D_ctrl->Pose6D_IK_Z_TD,R_6D_ctrl->Pose6D_IK.Z);
					R_6D_ctrl->Pose6D_IK.Z = R_6D_ctrl->Pose6D_IK_Z_TD.x;
					MoveL(R_6D_ctrl,arm_control_loop,arm_key);
		}
		else if(chassis.arm_mode == NX_CONTROL_MODE)
		{
				arm_control_loop->motor_1_position = rxData[0];
				arm_control_loop->motor_2_position = rxData[1];
				arm_control_loop->motor_3_position = rxData[2];
				arm_control_loop->motor_4_position = rxData[3];
				arm_control_loop->motor_5_position = rxData[4];
				arm_control_loop->motor_6_position = rxData[5];
		}
		arm_control_loop->motor_YAW_data.position_set = arm_control_loop->motor_1_position;
}

void arm_check_get_position(arm_control_t *check_position)
{
		if(check_position->motor_2_position - (arm_message.target_position[0]) < 0.2f &&
			 check_position->motor_2_position - (arm_message.target_position[0]) > -0.2f &&
		   check_position->motor_3_position - (arm_message.target_position[1]) < 0.1f &&
			 check_position->motor_3_position - (arm_message.target_position[1]) > -0.1f &&
		   check_position->motor_4_position - (arm_message.target_position[2]) < 0.1f &&
			 check_position->motor_4_position - (arm_message.target_position[2]) > -0.1f &&
		   check_position->motor_5_position - (arm_message.target_position[3]) < 0.2f &&
			 check_position->motor_5_position - (arm_message.target_position[3]) > -0.2f &&
		   check_position->motor_6_position - (arm_message.target_position[4]) < 0.15f &&
			 check_position->motor_6_position - (arm_message.target_position[4]) > -0.15f )
		{
				check_position->arm_get_position_flag++;
		}
		else
		{
				check_position->arm_get_position_flag = 0;
		}
}