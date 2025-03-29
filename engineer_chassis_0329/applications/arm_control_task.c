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
		R_6D_ctrl->Joint[0].angleLimitMax =  +2.90f;
		R_6D_ctrl->Joint[0].angleLimitMin =  -2.85f;
		
		R_6D_ctrl->Joint[1].angleLimitMax =  -0.329f;//3.1f;
		R_6D_ctrl->Joint[1].angleLimitMin =  -3.129f;//0.14f;
		
		R_6D_ctrl->Joint[2].angleLimitMax =  1.558f;//1.57f;
		R_6D_ctrl->Joint[2].angleLimitMin =  -1.242f;//-1.23f;
		
		R_6D_ctrl->Joint[3].angleLimitMax =  2.3f;
		R_6D_ctrl->Joint[3].angleLimitMin =  -1.9f;
		
		R_6D_ctrl->Joint[4].angleLimitMax =  PI;
		R_6D_ctrl->Joint[4].angleLimitMin = -PI;
		
		R_6D_ctrl->Joint[5].angleLimitMax =  PI;
		R_6D_ctrl->Joint[5].angleLimitMin = -PI;
		
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
//		TD_init(&R_6D_ctrl->Pose6D_IK_Z_TD, 50.0f, 2.0f, 0.002f, R_6D_ctrl->Pose6D_IK.Z);
		
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
			
		TD_init(&arm_control.arm_1_TD,12.0f,2.0f,0.002f,arm_control.motor_YAW_data.DM_motor_measure->motor_position);
}

void arm_feedback_update( arm_control_t *arm_control_position, Robotic_6DOF_control_t *R_6D_ctrl)
{
		//正解算输入
		R_6D_ctrl->Joint6D_FK.theta[0] = 	arm_message.target_position[0];
		R_6D_ctrl->Joint6D_FK.theta[1] =  -arm_message.target_position[1]-0.279f;
		R_6D_ctrl->Joint6D_FK.theta[2] =  arm_message.target_position[2]-1.292f;
		R_6D_ctrl->Joint6D_FK.theta[3] =  arm_message.target_position[3];
		R_6D_ctrl->Joint6D_FK.theta[4] =  arm_message.target_position[4];
		R_6D_ctrl->Joint6D_FK.theta[5] =  arm_message.target_position[5]+0.74f;
		//正解算调用
		SolveFK(&R_6D_ctrl->Robotic_6D, &R_6D_ctrl->Joint6D_FK,&R_6D_ctrl->Pose6D_FK);	
	
	 if(Quaterniont_Mode)
	 {
				float arm_pose_q_temp[4];
				const float q0[4] = {0.707,0,0,-0.707};//绕z轴逆时针转90度
				float q_multiply_result[4];
		 
				R_6D_ctrl->Pose6D_IK.X = arm_pose.x*50.0f-179.864f    -185.0f;//2025/3/26测试用的偏移量
				R_6D_ctrl->Pose6D_IK.Y = arm_pose.y*50.0f;
				R_6D_ctrl->Pose6D_IK.Z = -arm_pose.z*50.0f+469.576f    +50.0f;
				
				for(int i = 0; i < 4; i++)
				{
					arm_pose_q_temp[i] = arm_pose.q[i];
				}
				
				QuaternionMultiply(q0, arm_pose_q_temp, q_multiply_result);
		 
				R_6D_ctrl->Pose6D_IK.Q[0] = q_multiply_result[0];
				R_6D_ctrl->Pose6D_IK.Q[1] = q_multiply_result[1];
				R_6D_ctrl->Pose6D_IK.Q[2] = q_multiply_result[2];
				R_6D_ctrl->Pose6D_IK.Q[3] = q_multiply_result[3]; 
	 }
		if(chassis.arm_mode == 1 && chassis.suker_key_flag == 0)
		{
			TD_set_r(&arm_control_position->arm_1_TD,10.0f);
		}
		else if((chassis.arm_mode == 0 || chassis.arm_mode == 2) && chassis.suker_key_flag == 0) 
		{
			TD_set_r(&arm_control_position->arm_1_TD,5.0f);
		}
		else
		{
			TD_set_r(&arm_control_position->arm_1_TD,5.0f);
		}
}

void arm_control_set(arm_control_t *arm_control_set, all_key_t *arm_key)
{
				if(chassis.chassis_mode == 0)
		{
				TD_set_x(&arm_control_set->arm_1_TD,arm_control_set->motor_YAW_data.DM_motor_measure->motor_position);
		}
		else
		{
				TD_calc(&arm_control_set->arm_1_TD, arm_control_set->motor_1_position);
				
				arm_control_set->motor_YAW_data.position_set = (fp32)(arm_control_set->arm_1_TD.x);					
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

Joint6D_t deltaJoints;


void MoveL(Robotic_6DOF_control_t *R_6D_ctrl, arm_control_t *arm_control_set, all_key_t *arm_key)
{
		Joint6D_t tmp;
		bool valid[8];
		bool SolveIK_Success;
		bool new_joint_flag = false;
		//机械臂逆解算
		SolveIK_Success = SolveIK(&R_6D_ctrl->Robotic_6D, &R_6D_ctrl->Pose6D_IK,
															&R_6D_ctrl->output_solvers_IK, &Last_Joint6D, Quaterniont_Mode);

		if(!SolveIK_Success)
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
						
						if (valid[i]) R_6D_ctrl->ValidCnt++;
				}                                                                                                 
				
				if (R_6D_ctrl->ValidCnt)
				{
						float min = 1000.0f;
						uint8_t indexConfig = 0;
						for (int i = 0; i < 8; i++)
						{
								if (valid[i])
								{

										for (int j = 0; j < 6; j++)
										{
												R_6D_ctrl->Joint_Valid[i][j] = R_6D_ctrl->output_solvers_IK.theta[i][j];
												//Last_Joint6D.theta[j] = R_6D_ctrl->output_solvers_IK.theta[i][j];
												tmp.theta[j]= R_6D_ctrl->Joint6D_FK.theta[j] - R_6D_ctrl->output_solvers_IK.theta[i][j];
										}
										float maxAngle = AbsMaxOf6(tmp);
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
						R_6D_ctrl->Joint_Final[0] 	= 	R_6D_ctrl->output_solvers_IK.theta[indexConfig][0];
						R_6D_ctrl->Joint_Final[1]   =   R_6D_ctrl->output_solvers_IK.theta[indexConfig][1];
						R_6D_ctrl->Joint_Final[2]		=	  R_6D_ctrl->output_solvers_IK.theta[indexConfig][2];
						R_6D_ctrl->Joint_Final[3] 	=		R_6D_ctrl->output_solvers_IK.theta[indexConfig][3];
						R_6D_ctrl->Joint_Final[4] 	=		R_6D_ctrl->output_solvers_IK.theta[indexConfig][4];
						R_6D_ctrl->Joint_Final[5] 	=		R_6D_ctrl->output_solvers_IK.theta[indexConfig][5];		

						for (int j = 0; j < 6; j++)
						{
								Last_Joint6D.theta[j] = R_6D_ctrl->Joint_Final[j];
						}
						
						arm_control_set->motor_1_position 				= 	R_6D_ctrl->output_solvers_IK.theta[indexConfig][0];
						arm_control_set->motor_2_position         =   -(R_6D_ctrl->output_solvers_IK.theta[indexConfig][1]+0.279);
						arm_control_set->motor_3_position					=	  R_6D_ctrl->output_solvers_IK.theta[indexConfig][2]+1.292;
						arm_control_set->motor_4_position 				=		R_6D_ctrl->output_solvers_IK.theta[indexConfig][3];
						arm_control_set->motor_5_position 				=	  -R_6D_ctrl->output_solvers_IK.theta[indexConfig][4]+0.74;
						arm_control_set->motor_6_position 				=		-R_6D_ctrl->output_solvers_IK.theta[indexConfig][5];
				}
				else
				{
						for(int j=0;j<6;j++)
						{
									R_6D_ctrl->Joint_Final[j] = 0.0f;
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
//					TD_calc(&R_6D_ctrl->Pose6D_IK_Z_TD,R_6D_ctrl->Pose6D_IK.Z);
//					R_6D_ctrl->Pose6D_IK.Z = R_6D_ctrl->Pose6D_IK_Z_TD.x;
					MoveL(R_6D_ctrl,arm_control_loop,arm_key);
		}
		else if(chassis.arm_mode == NX_CONTROL_MODE)
		{
				arm_control_loop->motor_1_position = pc_receive_msg.rx_data.motor1_position;
				arm_control_loop->motor_2_position = pc_receive_msg.rx_data.motor2_position;
				arm_control_loop->motor_3_position = pc_receive_msg.rx_data.motor3_position;
				arm_control_loop->motor_4_position = pc_receive_msg.rx_data.motor4_position;
				arm_control_loop->motor_5_position = pc_receive_msg.rx_data.motor5_position;
				arm_control_loop->motor_6_position = pc_receive_msg.rx_data.motor6_position;
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
