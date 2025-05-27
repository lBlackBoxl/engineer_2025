#include "boards_communicate_task.h"
#include "Butterworth_Filter.h"
#include "arm_control_task.h"
#include "can_communicate.h"
#include "controller.h"
#include "arm_math.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "bsp_dwt.h"
#include "timers.h"
#include "stdio.h"
#include "task.h"
#include "main.h"
#include "TD.h"
#include "tim.h"

static void arm_control_init(arm_t *arm_control_init);
static void arm_feedback_update(arm_t *arm_feedback);
static void arm_set_mode(arm_t *arm_set_mode);
static void arm_set_control(arm_t *arm_set_control);
static void arm_control_loop(arm_t *arm_control_loop);

arm_t arm;
uint8_t  reset_3508_switch;
uint32_t reset_3508_count[2];
uint32_t reset_2006_count[2];
uint32_t reposition_count = 0;
bool_t reposition_flag = 0;
float32_t g_2006 = 249.0f;
float32_t mode2_cnt = 0;
int joint6_round;
fp32 roll_angle_set;
fp32 last_roll_angle_set;
fp32 joint6_position;

void arm_task(void const * argument)
{	
		//等待陀螺仪任务更新陀螺仪数据
		vTaskDelay(ARM_TASK_INIT_TIME);
		//初始化arm电机
		arm_control_init(&arm);
		//定义dwt定时器计时变量
		uint32_t system_clock = osKernelSysTick();

		while(1)
		{
				arm.dt = DWT_GetDeltaT(&arm.DWT_Count);	

				arm_set_mode(&arm);		
				arm_feedback_update(&arm);
				arm_set_control(&arm);
				arm_control_loop(&arm);
			  //1-3508
				//2-4达妙发送指令 重力补偿未写
				if(arm.chassis_mode == 0)
				{
						CAN_cmd_4310_disable(DM_M2_TX_ID, hcan1);	
						DWT_Delay(0.0003f);
						CAN_cmd_4310_disable(DM_M3_TX_ID, hcan1);	
						DWT_Delay(0.0003f);
						CAN_cmd_4310_disable(DM_M4_TX_ID, hcan1);	
						DWT_Delay(0.0003f);
						joint6_round = 0;
						joint6_position = board_message.target_position[5];
						arm.yaw_angle_set = joint6_position;
						arm.motor_2006_data[0].round_cnt = 0;
						arm.motor_2006_data[1].round_cnt = 0;
						arm.motor_2006_data[0].angle = 0;
						arm.motor_2006_data[1].angle = 0;
				}
				else if(arm.chassis_mode != 0)
				{				
						if(arm.motor_DM_data[0].DM_motor_measure->motor_enabled == 0)
								CAN_cmd_4310_enable(DM_M2_TX_ID, hcan1);					
						else if(arm.motor_DM_data[0].DM_motor_measure->motor_enabled == 1)	
								CAN_cmd_4310_mit(arm.motor_DM_data[0].position_set,0.0f,2000.0f,2.0f,0.0f,DM_M2_TX_ID, hcan1);
						else
								CAN_cmd_4310_disable(DM_M2_TX_ID, hcan1);	
								DWT_Delay(0.0003f);	
						
						if(arm.motor_DM_data[1].DM_motor_measure->motor_enabled == 0)
								CAN_cmd_4310_enable(DM_M3_TX_ID, hcan1);					
						else if(arm.motor_DM_data[1].DM_motor_measure->motor_enabled == 1)	
								CAN_cmd_4310_mit(arm.motor_DM_data[1].position_set,0.0f,7000.0f,5.0f,0.0f,DM_M3_TX_ID, hcan1);
						else
								CAN_cmd_4310_disable(DM_M3_TX_ID, hcan1);	
								DWT_Delay(0.0003f);	
						
						if(arm.motor_DM_data[2].DM_motor_measure->motor_enabled == 0)
								CAN_cmd_4310_enable(DM_M4_TX_ID, hcan1);					
						else if(arm.motor_DM_data[2].DM_motor_measure->motor_enabled == 1)	
								CAN_cmd_4310_mit(arm.motor_DM_data[2].position_set,0.0f,85.0f,1.0f,0.0f,DM_M4_TX_ID, hcan1);
						else
								CAN_cmd_4310_disable(DM_M4_TX_ID, hcan1);	
								DWT_Delay(0.0003f);	
				}
				// 5-6-2006发送电流指令
				CAN_cmd_2006(arm.motor_2006_data[0].give_current,arm.motor_2006_data[1].give_current);			
				osDelay(1);
		}	
}

void arm_control_init(arm_t *arm_control_init)
{	
    if (arm_control_init == NULL)
    {
        return;
    }		
		//初始化电机数据指针
		for(int i = 0; i < 3; i++)
		{
				arm_control_init->motor_DM_data[i].DM_motor_measure = return_4310_measure(i + 1);
		}
		for(int i = 0; i < 2; i++)
		{
				arm_control_init->motor_2006_data[i].motor_measure = get_chassis_motor_point(i);
		}
		//初始化PID
		PID_Init(&arm_control_init->motor_2006_speed_pid[0], M2006_ROTATE_SPEED_PID_MAX_OUT, M2006_ROTATE_SPEED_PID_MAX_IOUT,0.0f,M2006_SPEED_PID_KP_0_INIT, M2006_SPEED_PID_KI_INIT, M2006_SPEED_PID_KD_INIT,0.0f,0.0f,0.00106103295394596890512589175582f,0.0f,5,0x11);
		PID_Init(&arm_control_init->motor_2006_speed_pid[1], M2006_ROTATE_SPEED_PID_MAX_OUT, M2006_ROTATE_SPEED_PID_MAX_IOUT,0.0f,M2006_SPEED_PID_KP_1_INIT, M2006_SPEED_PID_KI_INIT, M2006_SPEED_PID_KD_INIT,0.0f,0.0f,0.00106103295394596890512589175582f,0.0f,5,0x11);
		PID_Init(&arm_control_init->yaw_angle_pid,ARM_YAW_PID_MAX_OUT,ARM_YAW_PID_MAX_IOUT,0.0f,ARM_YAW_PID_KP, ARM_YAW_PID_KI, ARM_YAW_PID_KD,0.0f,0.0f,0.0f,0.0f,4,0x01);
		PID_Init(&arm_control_init->roll_angle_pid,ARM_ROLL_PID_MAX_OUT,ARM_ROLL_PID_MAX_IOUT,0.0f,ARM_ROLL_PID_KP, ARM_ROLL_PID_KI, ARM_ROLL_PID_KD,0.0f,0.0f,0.0f,0.0f,4,0x01);

		//达妙电机需要先发送消息才有反馈帧
		CAN_cmd_4310_disable(DM_M2_TX_ID, hcan1);
		osDelay(1);
		CAN_cmd_4310_disable(DM_M3_TX_ID, hcan1);
		osDelay(1);
		CAN_cmd_4310_disable(DM_M4_TX_ID, hcan1);
		osDelay(1);
		//数据更新
		arm_feedback_update(arm_control_init);
		//初始化电机相关数据
		arm_control_init->arm_mode = 0;	
		arm_control_init->chassis_mode = 0;
		arm_control_init->motor_DM_data[0].position_set = arm_control_init->motor_DM_data[0].DM_motor_measure->motor_position;
		arm_control_init->motor_DM_data[1].position_set = arm_control_init->motor_DM_data[1].DM_motor_measure->motor_position;	
		arm_control_init->motor_DM_data[2].position_set = arm_control_init->motor_DM_data[2].DM_motor_measure->motor_position;	
		
		arm_control_init->motor_2006_mode = 0;
		for(int16_t i = 0; i < 2; i++)
		{
			arm_control_init->motor_2006_data[i].offset_ecd = arm_control_init->motor_2006_data[i].motor_measure->ecd;
			arm_control_init->motor_2006_data[i].angle_set = arm_control_init->motor_2006_data[i].angle;
			arm_control_init->motor_2006_data[i].speed_set = 0;
		}
		//初始化跟踪微分器TD
		TD_init(&arm_control_init->arm_2_TD,3.0f,2.0f,0.002f,arm_control_init->motor_DM_data[1].DM_motor_measure->motor_position);
		TD_init(&arm_control_init->arm_3_TD,3.0f,2.0f,0.002f,arm_control_init->motor_DM_data[2].DM_motor_measure->motor_position);
		TD_init(&arm_control_init->arm_4_TD,3.0f,2.0f,0.002f,arm_control_init->motor_DM_data[3].DM_motor_measure->motor_position);
		TD_init(&arm_control_init->arm_5_TD,3.0f,2.0f,0.002f,arm_control_init->roll_angle);
		TD_init(&arm_control_init->arm_6_TD,3.0f,2.0f,0.002f,arm_control_init->yaw_angle);		
}

void arm_set_mode(arm_t *arm_set_mode)
{	
    if (arm_set_mode == NULL)
    {
        return;
    }	
		
		//更新底盘上传的标志位
		arm_set_mode->arm_last_mode = arm_set_mode->arm_mode;	
		arm_set_mode->arm_mode = board_message.arm_mode;		
		arm_set_mode->chassis_last_mode = arm_set_mode->chassis_mode;	
		arm_set_mode->chassis_mode = board_message.chassis_mode;	
		arm_set_mode->suker_mode = board_message.suker_mode;
    	arm_set_mode->move_mode = board_message.move_mode;
		//根据底盘标志位更新1-3508和5&6-2006标志位
		if(arm_set_mode->chassis_mode == 0)
		{
				arm_set_mode->motor_2006_mode = 0;
		}
		else if(arm_set_mode->chassis_mode != 0 && arm_set_mode->chassis_last_mode == 0)
		{
				arm_set_mode->motor_2006_mode = 1;
		}
		//根据初始化情况更新2006标志位
		if(arm_set_mode->motor_2006_mode == 1)
		{
			if(arm_set_mode->motor_2006_last_mode != 1)
			{
				Set_KP(&arm_set_mode->motor_2006_speed_pid[0],M2006_SPEED_PID_KP_0);
				Set_KP(&arm_set_mode->motor_2006_speed_pid[1],M2006_SPEED_PID_KP_0);
			}
				if(fabs(arm_set_mode->motor_2006_data[0].speed) < 0.2f)
						reset_2006_count[0] ++;
				else
						reset_2006_count[0] = 0;
				if(fabs(arm_set_mode->motor_2006_data[1].speed) < 0.2f)
						reset_2006_count[1] ++;
				else
						reset_2006_count[1] = 0;										
				if (reset_2006_count[0] > 150 && reset_2006_count[1] > 150)
				{
						reset_2006_count[0] = 0;
						reset_2006_count[1] = 0;
						arm_set_mode->motor_2006_mode = 2;
						arm_set_mode->motor_2006_data[0].round_cnt = 0;
						arm_set_mode->motor_2006_data[1].round_cnt = 0;
						arm_set_mode->motor_2006_data[0].offset_ecd = arm_set_mode->motor_2006_data[0].motor_measure->ecd;
						arm_set_mode->motor_2006_data[1].offset_ecd = arm_set_mode->motor_2006_data[1].motor_measure->ecd;
						TD_set_x(&arm_set_mode->arm_5_TD,arm_set_mode->roll_angle);
						TD_set_x(&arm_set_mode->arm_6_TD,arm_set_mode->yaw_angle);
						
				}
		}
		//初始化结束后进入可控状态
		//测试
		if(arm_set_mode->motor_2006_mode == 2)
		{
			if(arm_set_mode->motor_2006_last_mode != 2)
				{
							arm_set_mode->motor_2006_speed_pid[0].Kp = 2000;
							arm_set_mode->motor_2006_speed_pid[1].Kp = 2000;
				}
			arm_set_mode->motor_2006_mode = 3;
		}
		
//		if(arm_set_mode->motor_3508_mode == 2 && arm_set_mode->motor_2006_mode == 2 
//			&& fabs(arm_set_mode->motor_3508_data.angle - 20.0f) < 0.5f )
//		{
//			if(arm_set_mode->motor_2006_last_mode != 2)
//			{
//				Set_KP(&arm_set_mode->motor_2006_speed_pid[0],M2006_SPEED_PID_KP_1);
//				Set_KP(&arm_set_mode->motor_2006_speed_pid[1],M2006_SPEED_PID_KP_1);
//				mode2_cnt = DWT_GetTimeline_ms();
//			}
//			if(DWT_GetTimeline_ms() - mode2_cnt > 300.0f)
//			{
//				arm_set_mode->motor_3508_mode = 3;
//				arm_set_mode->motor_2006_mode = 3;
//			}
//		}	
		arm_set_mode->motor_2006_last_mode = arm_set_mode->motor_2006_mode;
}

void arm_feedback_update(arm_t *arm_feedback)
{
		if(arm_feedback == NULL)
		{
				return;
		}
		
		//更新5&6-2006数据
		for(int16_t i = 0; i < 2; i++)
		{
			arm_feedback->motor_2006_data[i].speed = arm_feedback->motor_2006_data[i].motor_measure->speed_rpm * RPM_TO_RAD_S * REDUCTION_RATIO_2006;
			arm_feedback->motor_2006_data[i].angle = (arm_feedback->motor_2006_data[i].round_cnt * ECD_RANGE 
					+ arm_feedback->motor_2006_data[i].motor_measure->ecd - arm_feedback->motor_2006_data[i].offset_ecd) * MOTOR_ECD_TO_ANGLE_2006;	
		}
		arm.yaw_angle = (arm.motor_2006_data[0].angle + arm.motor_2006_data[1].angle)/(2 * YAW_TO_2006);
	  arm.roll_angle = (arm.motor_2006_data[0].angle - arm.motor_2006_data[1].angle)/(2 * ROLL_TO_2006);//转化成末端两轴角度

		//根据模式更改TD
		if(arm_feedback->chassis_mode == 0)
		{
			reposition_count = 0;
			reposition_flag = 0;
			TD_set_r(&arm_feedback->arm_2_TD,3.0f);
			TD_set_r(&arm_feedback->arm_3_TD,3.0f);
			TD_set_r(&arm_feedback->arm_4_TD,3.0f);
			TD_set_r(&arm_feedback->arm_5_TD,3.0f);
			TD_set_r(&arm_feedback->arm_6_TD,3.0f);
		}
		else
		{
			if(reposition_count < 5000)
			{
				reposition_count++;
			}
			else
			{
				reposition_flag = 1;
				reposition_count = 0;
			}
			
			if(reposition_flag == 1)
			{
				if(arm_feedback->arm_mode == 1 && arm_feedback->suker_mode == 0)
				{		
						if(reposition_count < 500)
						{
							reposition_count++;
						}
						else
						{
							TD_set_r(&arm_feedback->arm_2_TD,5.0f);
							TD_set_r(&arm_feedback->arm_3_TD,5.0f);
							TD_set_r(&arm_feedback->arm_4_TD,5.0f);
							TD_set_r(&arm_feedback->arm_5_TD,5.0f);
							TD_set_r(&arm_feedback->arm_6_TD,5.0f);
						}
				}
				else if((arm_feedback->arm_mode == 0 || arm_feedback->arm_mode == 2 )&& arm_feedback->suker_mode == 1)
				{
						TD_set_r(&arm_feedback->arm_2_TD,7.0f);
						TD_set_r(&arm_feedback->arm_3_TD,7.0f);
						TD_set_r(&arm_feedback->arm_4_TD,7.0f);
						TD_set_r(&arm_feedback->arm_5_TD,7.0f);
						TD_set_r(&arm_feedback->arm_6_TD,7.0f);
						reposition_count = 0;
				}	
				else
				{
						TD_set_r(&arm_feedback->arm_2_TD,7.0f);
						TD_set_r(&arm_feedback->arm_3_TD,7.0f);
						TD_set_r(&arm_feedback->arm_4_TD,7.0f);
						TD_set_r(&arm_feedback->arm_5_TD,7.0f);
						TD_set_r(&arm_feedback->arm_6_TD,7.0f);	
						reposition_count = 0;
				}
			}
		}
}

void arm_set_control(arm_t *arm_set_control)
{
		if(arm_set_control == NULL)
		{
				return;
		}
		//2-4
		if(arm_set_control->chassis_mode == 0)
		{
				TD_set_x(&arm_set_control->arm_2_TD,arm_set_control->motor_DM_data[0].DM_motor_measure->motor_position);
				TD_set_x(&arm_set_control->arm_3_TD,arm_set_control->motor_DM_data[1].DM_motor_measure->motor_position);
				TD_set_x(&arm_set_control->arm_4_TD,arm_set_control->motor_DM_data[2].DM_motor_measure->motor_position);
		}
		else
		{
				TD_calc(&arm_set_control->arm_2_TD, board_message.target_position[1]);
				TD_calc(&arm_set_control->arm_3_TD, board_message.target_position[2]);
				TD_calc(&arm_set_control->arm_4_TD, board_message.target_position[3]);
				
				arm_set_control->motor_DM_data[0].position_set = (fp32)(arm_set_control->arm_2_TD.x);					
				arm_set_control->motor_DM_data[1].position_set = (fp32)(arm_set_control->arm_3_TD.x);					
				arm_set_control->motor_DM_data[2].position_set = (fp32)(arm_set_control->arm_4_TD.x);
		}
		//5-6		
		last_roll_angle_set = roll_angle_set;
		roll_angle_set = board_message.target_position[5];
		if(roll_angle_set - last_roll_angle_set < -PI)
		{
			joint6_round --;
		}
		else if(roll_angle_set - last_roll_angle_set > PI)
		{
			joint6_round ++;
		}
		joint6_position = board_message.target_position[5] - 2 * PI * joint6_round;
		if(arm_set_control->motor_2006_mode == 0)
		{
				TD_set_x(&arm_set_control->arm_5_TD,arm_set_control->roll_angle);
				TD_set_x(&arm_set_control->arm_6_TD,joint6_position);	
		}
		else if(arm_set_control->motor_2006_mode == 1)
		{			
				TD_set_x(&arm_set_control->arm_5_TD,arm_set_control->roll_angle);
				TD_set_x(&arm_set_control->arm_6_TD,joint6_position);		
				arm_set_control->motor_2006_data[0].speed_set =  -2.0f;
				arm_set_control->motor_2006_data[1].speed_set =   2.0f;
		}
		else if(arm_set_control->motor_2006_mode == 2)
		{
				TD_set_x(&arm_set_control->arm_5_TD,arm_set_control->roll_angle);
				TD_set_x(&arm_set_control->arm_6_TD,joint6_position);					
		}
		else
		{
				//锥齿轮的处理
				TD_calc(&arm_set_control->arm_5_TD, board_message.target_position[4]);
				TD_calc(&arm_set_control->arm_6_TD, joint6_position);
				arm_set_control->roll_angle_set = arm_set_control->arm_5_TD.x;
				arm_set_control->yaw_angle_set = arm_set_control->arm_6_TD.x;
		}
		
		if(arm_set_control -> move_mode == 4)
		{
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,1200);//500-2000
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,1200);//500-2000
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,1200);//500-2000
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,1200);//500-2000
		}
		else
		{
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,1700);//500-2000
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,1700);//500-2000
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,1700);//500-2000
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,1700);//500-2000
		}
}

void arm_control_loop(arm_t *arm_control_loop)
{		
		//5&6-2006
		if(arm_control_loop->motor_2006_mode == 0)
		{
				arm_control_loop->motor_2006_data[0].give_current = 0;
				arm_control_loop->motor_2006_data[1].give_current = 0;
		}
		else if(arm_control_loop->motor_2006_mode == 1)
		{
				PID_Calculate(&arm_control_loop->motor_2006_speed_pid[0], arm_control_loop->motor_2006_data[0].speed, arm_control_loop->motor_2006_data[0].speed_set);
//				arm_control_loop->motor_2006_data[0].give_current = int16_constrain((int16_t)(arm_control_loop->motor_2006_speed_pid[0].Output),-6000,6000);
				arm_control_loop->motor_2006_data[0].give_current = (int16_t)(arm_control_loop->motor_2006_speed_pid[0].Output);
				PID_Calculate(&arm_control_loop->motor_2006_speed_pid[1], arm_control_loop->motor_2006_data[1].speed, arm_control_loop->motor_2006_data[1].speed_set);
				arm_control_loop->motor_2006_data[1].give_current = (int16_t)(arm_control_loop->motor_2006_speed_pid[1].Output);
//				arm_control_loop->motor_2006_data[1].give_current = int16_constrain((int16_t)(arm_control_loop->motor_2006_speed_pid[0].Output),-6000,6000);

		}
		else if(arm_control_loop->motor_2006_mode == 2)
		{
				arm_control_loop->motor_2006_data[0].give_current = 0;
				arm_control_loop->motor_2006_data[1].give_current = 0;			
		}
		else if(arm_control_loop->motor_2006_mode == 3)
		{
				int16_t current_2006_tem[2];
			  if(fabs(arm_control_loop->yaw_angle - arm_control_loop->yaw_angle_set) < 0.1f)
				{
						arm_control_loop->yaw_angle_pid.Output = 0;
				}
				else
				{
						PID_Calculate_Angle(&arm_control_loop->yaw_angle_pid,arm_control_loop->yaw_angle,arm_control_loop->yaw_angle_set);
				}
				if(fabs(arm_control_loop->roll_angle - arm_control_loop->roll_angle_set) < 0.2f)
				{
						arm_control_loop->roll_angle_pid.Output = 0;
				}
				{
						PID_Calculate(&arm_control_loop->roll_angle_pid,arm_control_loop->roll_angle,arm_control_loop->roll_angle_set);
				}
				
				arm_control_loop->motor_2006_data[0].speed_set = arm_control_loop->roll_angle_pid.Output + arm_control_loop->yaw_angle_pid.Output;
				PID_Calculate(&arm_control_loop->motor_2006_speed_pid[0], arm_control_loop->motor_2006_data[0].speed, arm_control_loop->motor_2006_data[0].speed_set);
		
				arm_control_loop->motor_2006_data[1].speed_set = - arm_control_loop->roll_angle_pid.Output + arm_control_loop->yaw_angle_pid.Output;;
				PID_Calculate(&arm_control_loop->motor_2006_speed_pid[1], arm_control_loop->motor_2006_data[1].speed, arm_control_loop->motor_2006_data[1].speed_set);
			
			if(board_message.suker_mode == 0)
				{				
					current_2006_tem[0] = (int16_t)(arm_control_loop->motor_2006_speed_pid[0].Output);
					current_2006_tem[1] = (int16_t)(arm_control_loop->motor_2006_speed_pid[1].Output);
				}
			else if(board_message.suker_mode == 1)
				{
					current_2006_tem[0] = (int16_t)(arm_control_loop->motor_2006_speed_pid[0].Output + g_2006 * cosf(arm_control_loop->yaw_angle)*cosf(arm_control_loop->roll_angle - 1.92f));
					current_2006_tem[1] = (int16_t)(arm_control_loop->motor_2006_speed_pid[1].Output - g_2006 * cosf(arm_control_loop->yaw_angle)*cosf(arm_control_loop->roll_angle - 1.92f));  									
				}
					arm_control_loop->motor_2006_data[0].give_current = int16_constrain(current_2006_tem[0],-10000,10000);
					arm_control_loop->motor_2006_data[1].give_current = int16_constrain(current_2006_tem[1],-10000,10000);  

		}						
}