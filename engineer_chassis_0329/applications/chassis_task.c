/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             ���̿�������

  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "FreeRTOS.h"
#include "chassis_task.h"
#include "init.h"
#include "remote_control.h"
#include "can_communicate.h" 
#include "referee_usart_task.h"
#include "referee.h"
#include "task.h"
#include "cmsis_os.h"
#include "main.h"
#include "can.h"
#include "bsp_usart.h"
#include "arm_control_task.h"

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

static void chassis_init(all_key_t *chassis_key_init, chassis_t *chassis_init);
		
static void chassis_key_check(all_key_t *chassis_key_check);

static void chassis_set_mode(all_key_t *chassis_set_key, chassis_t *chassis_set_mode);

static void chassis_feedback_update(chassis_t *chassis_update);

static void chassis_set_contorl(chassis_t *chassis_control);

static void chassis_control_loop(chassis_t *chassis_control_loop);	
		
chassis_t chassis;
int arm_restart_flag;
bool_t  clamp_flag;
uint8_t clamp_mode;
int16_t clamp_reset_count;		
		
/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{
		vTaskDelay(CHASSIS_TASK_INIT_TIME);
	
		chassis_init(&all_key, &chassis);
		
		while(1)
		{
				chassis.dt = DWT_GetDeltaT(&chassis.DWT_Count);
				//�������
				chassis_key_check(&all_key);
			  //���õ��̿���ģʽ
        chassis_set_mode(&all_key, &chassis);
			  //�������ݸ���
        chassis_feedback_update(&chassis);
			  //���̿���������
        chassis_set_contorl(&chassis);
			  //���̿���PID����
        chassis_control_loop(&chassis);
				
				//ͨ������ģʽ�����������
				if(chassis.chassis_mode == NO_POWER_MODE)
				{			
						CAN_cmd_chassis(0, 0, 0, 0);
//						CAN_cmd_chassis_clamp(0);
				}
				else if(chassis.chassis_mode == RUN_MODE || chassis.chassis_mode == STORAGE_MODE)
				{
						CAN_cmd_chassis(chassis.motor_chassis[0].give_current,  chassis.motor_chassis[1].give_current, 
														 chassis.motor_chassis[2].give_current,  chassis.motor_chassis[3].give_current);
						CAN_cmd_chassis_clamp(chassis.motor_clamp.give_current);
//					CAN_cmd_4310_PV(chassis.motor_DM_data[0].position_set,chassis.motor_DM_data[0].speed_set,DM_4310_M1_TX_ID);//4310λ���ٶ�ģʽ���
//					DWT_Delay(0.0003f);
//					CAN_cmd_4310_PV(chassis.motor_DM_data[1].position_set,chassis.motor_DM_data[1].speed_set,DM_4310_M2_TX_ID);
//					DWT_Delay(0.0003f);
//						CAN_cmd_4310_mit(chassis.motor_DM_data.position_set,0.0f,200.0f,5.0f,0.0f,DM_4310_M1_TX_ID,hcan2);
//						DWT_Delay(0.0003f);
//					CAN_cmd_4310_mit(chassis.motor_DM_data[1].position_set,0.0f,200.0f,5.0f,0.0f,DM_4310_M2_TX_ID);
//					DWT_Delay(0.0003f);
				}
				
        //ϵͳ��ʱ
				osDelay(2);

		}
}

/**
  * @brief          ��ʼ��"chassis"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ��
  * @param[out]     chassis_init:"chassis"����ָ��.
  * @retval         none
  */

static void chassis_init(all_key_t *chassis_key_init, chassis_t *chassis_init)
{
	    if (chassis_init == NULL)
    {
        return;
    }
		
		chassis.error_info = 0;
		
		const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
		
		//���̿���״̬Ϊ����
		chassis_init->chassis_mode = NO_POWER_MODE;
		
		//��е��Ĭ�Ͽ���ģʽΪһ��ģʽ
		chassis_init->arm_mode = ONE_KEY_MODE;
		
		//��е��Ŀ��λ��ΪHOME
		chassis_init->move_mode = Home;
		
		//����Ĭ��Ϊ��
		chassis_init->suker_key_flag = 0;
		
		//��ȡң����ָ��
		chassis_init->chassis_RC = get_remote_control_point();
		
		//Ĭ�ϻ�е�۲���Ҫ����
		arm_restart_flag = 0;
    
		//�п�
		PID_Init(&chassis_init->clamp_motor_position_pid, CLAMP_POSITION_PID_MAX_OUT, CLAMP_POSITION_PID_MAX_KD,0.0f,
						CLAMP_POSITION_PID_KP, CLAMP_POSITION_PID_KI, CLAMP_POSITION_PID_KD,0.0f,0.0f,0.00106103295394596890512589175582f,0.0f,4,0x11);
		PID_Init(&chassis_init->clamp_motor_speed_pid, CLAMP_SPEED_PID_MAX_OUT, CLAMP_SPEED_PID_MAX_KD,0.0f,
						CLAMP_SPEED_PID_KP, CLAMP_SPEED_PID_KI, CLAMP_SPEED_PID_KD,0.0f,0.0f,0.00106103295394596890512589175582f,0.0f,5,0x11);	
		
		//��ȡ���̵������ָ�룬��ʼ��PID 
		for (uint8_t i = 0; i < 4; i++)
    {
				chassis_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_point(i);
				PID_Init(&chassis_init->chassis_motor_speed_pid[i], CHASSIS_M3508_SPEED_PID_MAX_OUT, CHASSIS_M3508_SPEED_PID_MAX_IOUT,0.0f,
								CHASSIS_M3508_SPEED_PID_KP, CHASSIS_M3508_SPEED_PID_KI, CHASSIS_M3508_SPEED_PID_KD,0.0f,0.0f,0.000795774715459476f,0.0f,2,0x11);
    }
		
		chassis_init->motor_clamp.motor_measure = get_chassis_motor_point(4);
//		//��ȡ̨�״�������
//		chassis_init->motor_DM_data.DM_motor_measure = return_4310_measure(0);
		
		//��ʼ���Ƕ�PID
    PID_Init(&chassis_init->chassis_yaw_pid, CHASSIS_YAW_PID_MAX_OUT, CHASSIA_YAW_PID_MAX_IOUT,0.0f,
							CHASSIS_YAW_PID_KP, CHASSIS_YAW_PID_KI, CHASSIS_YAW_PID_KD,0.0f,0.0f,0.00106103295394596890512589175582f,0.0f,2,0x11);	
		
		//��һ���˲�����б����������
    first_order_filter_init(&chassis_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

		//��������Ҫ�ȷ�����Ϣ���з���֡
//		CAN_cmd_4310_disable(DM_4310_M1_TX_ID, hcan2);
//		DWT_Delay(0.0003f);
//		CAN_cmd_4310_disable(DM_4310_M2_TX_ID);
//		DWT_Delay(0.0003f);	
		
		//����һ������
    chassis_feedback_update(chassis_init);
		
		//��ʼ��TD����΢����
		TD_init(&chassis_init->chassis_yaw_TD, 12.0f, 2.0f, 0.002f, chassis_init->yaw_angle);

		//��ʼ������
		key_init(&chassis_key_init->rotate_key_G,G);
		key_init(&chassis_key_init->rotate_key_V,V);
		key_init(&chassis_key_init->exchange_key,X);
		key_init(&chassis_key_init->yaw_plus_key,Q);
		key_init(&chassis_key_init->yaw_minus_key,E);
		key_init(&chassis_key_init->gou_dong_key,B);		
		key_init(&chassis_key_init->arm_restart_key1,C);
		key_init(&chassis_key_init->arm_restart_key2,SHIFT);
}


/**
  * @brief          ���õ��̿���ģʽ
  * @param[out]     chassis_mode:"chassis"����ָ��.
  * @retval         none
  */
static void chassis_set_mode(all_key_t *chassis_set_key, chassis_t *chassis_set_mode)
{
				if (chassis_set_mode == NULL)
		{
				return;
		}
		
		chassis_set_mode->last_chassis_mode = chassis_set_mode->chassis_mode;
		chassis_set_mode->last_arm_mode = chassis_set_mode->arm_mode;
		
		//�����˶�ģʽѡ��
		if(switch_is_down(chassis_set_mode->chassis_RC->rc.s[RC_S_RIGHT]))
		{
			chassis_set_mode->chassis_mode = NO_POWER_MODE;
		}
		else if(switch_is_mid(chassis_set_mode->chassis_RC->rc.s[RC_S_RIGHT]))
		{
			chassis_set_mode->chassis_mode = RUN_MODE;
		}
		else if(switch_is_up(chassis_set_mode->chassis_RC->rc.s[RC_S_RIGHT]))
		{
			chassis_set_mode->chassis_mode = STORAGE_MODE;
		}
		
		//��е���˶�ģʽѡ��
		if(switch_is_down(chassis_set_mode->chassis_RC->rc.s[RC_S_LEFT]))
		{
			chassis_set_mode->arm_mode = NX_CONTROL_MODE;
		}
		else if(switch_is_mid(chassis_set_mode->chassis_RC->rc.s[RC_S_LEFT]))
		{
			chassis_set_mode->arm_mode = ONE_KEY_MODE;
		}
		else if(switch_is_up(chassis_set_mode->chassis_RC->rc.s[RC_S_LEFT]))
		{
			chassis_set_mode->arm_mode = SELF_CONTROL_MODE;
		}
		
		//�п�
		if ((chassis_set_mode->chassis_mode == STORAGE_MODE || chassis_set_mode->chassis_mode == RUN_MODE) && chassis_set_mode->last_chassis_mode == NO_POWER_MODE)
		{
				clamp_mode = 1;
		}
		else if(chassis_set_mode->chassis_mode == NO_POWER_MODE)
		{
				clamp_mode = 0;
		}
		if(clamp_mode == 1)
		{
				if(fabs(chassis_set_mode->motor_clamp.speed) < 1.0f)
						clamp_reset_count++;
				else
						clamp_reset_count = 0;
		}
		if(clamp_mode == 1 && clamp_reset_count >= 150)
		{
				chassis_set_mode->motor_clamp.round_cnt = 0;
				chassis_set_mode->motor_clamp.offset_ecd = chassis_set_mode->motor_clamp.motor_measure->ecd;
				clamp_mode = 2;
				clamp_reset_count = 0;
		}	
		
		//��е���˶�Ŀ��ѡ��
		if(all_key.rotate_key_G.itself.mode != all_key.rotate_key_G.itself.last_mode)
		{		
				if(chassis_set_mode->move_mode == chassis_set_mode->last_move_mode && chassis_set_mode->last_move_mode != Au)
				{
					chassis_set_mode->move_mode = Au;
				}
				else if(chassis_set_mode->last_move_mode == Au)
				{
					chassis_set_mode->move_mode = Home;
				}
				chassis_set_mode->last_move_mode = chassis_set_mode->move_mode;
		}
		if(all_key.rotate_key_V.itself.mode != all_key.rotate_key_V.itself.last_mode)
		{
				if(chassis_set_mode->move_mode == chassis_set_mode->last_move_mode && chassis_set_mode->last_move_mode != Ag)
				{
					chassis_set_mode->move_mode = Ag;
				}
				else if(chassis_set_mode->last_move_mode == Ag)
				{
					chassis_set_mode->move_mode = Home;
				}
				chassis_set_mode->last_move_mode = chassis_set_mode->move_mode;
		}
		if(all_key.exchange_key.itself.mode != all_key.exchange_key.itself.last_mode)
		{
				if(chassis_set_mode->move_mode == chassis_set_mode->last_move_mode && chassis_set_mode->last_move_mode != Exchange)
				{
					chassis_set_mode->move_mode = Exchange;
				}
				else if(chassis_set_mode->last_move_mode == Exchange)
				{
					chassis_set_mode->move_mode = Home;
				}
				chassis_set_mode->last_move_mode = chassis_set_mode->move_mode;
		}
		if(all_key.gou_dong_key.itself.mode != all_key.gou_dong_key.itself.last_mode)
		{
				if(chassis_set_mode->move_mode == chassis_set_mode->last_move_mode && chassis_set_mode->last_move_mode != GouDong)
				{
					chassis_set_mode->move_mode = GouDong;
				}
				else if(chassis_set_mode->last_move_mode == GouDong)
				{
					chassis_set_mode->move_mode = Home;
				}
				chassis_set_mode->last_move_mode = chassis_set_mode->move_mode;
		}
//		//���̹ؽڵ��ģʽѡ��
//		if(switch_is_mid(chassis_mode->chassis_RC->rc.s[RC_S_LEFT]))
//		{
//			chassis_mode->joint_mode = GO_AHEAD_MODE;
//		}
//		else if(switch_is_down(chassis_mode->chassis_RC->rc.s[RC_S_LEFT]))
//		{
//			chassis_mode->joint_mode = GO_DOWN_MODE;
//		}
//		else
//		{
//			chassis_mode->joint_mode = GO_UP_MODE;
//		}
}

void chassis_key_check(all_key_t *chassis_key_check)
{
		key_itself_press_num(&(chassis_key_check->rotate_key_G),2);
		key_itself_press_num(&(chassis_key_check->rotate_key_V),2);
		key_itself_press_num(&(chassis_key_check->yaw_plus_key),2);
		key_itself_press_num(&(chassis_key_check->yaw_minus_key),2);
		key_itself_press_num(&(chassis_key_check->home_key),2);
		key_itself_press_num(&(chassis_key_check->gou_dong_key),2);
		key_itself_press_num(&(chassis_key_check->arm_restart_key1),2);
		key_itself_press_num(&(chassis_key_check->arm_restart_key2),2);
}

/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     chassis_update:"chassis"����ָ��.
  * @retval         none
  */
static void chassis_feedback_update(chassis_t *chassis_update)
{
    if (chassis_update == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //���µ���ٶ�
        chassis_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
    }

		//�п�
		chassis_update->motor_clamp.position = (chassis_update->motor_clamp.round_cnt * ECD_RANGE + chassis_update->motor_clamp.motor_measure->ecd - chassis_update->motor_clamp.offset_ecd) * MOTOR_ECD_TO_ANGLE_2006;
    chassis_update->motor_clamp.speed = chassis_update->motor_clamp.motor_measure->speed_rpm * RPM_TO_RAD_S * REDUCTION_RATIO_2006;	
		
		TD_set_r(&chassis_update->joint_TD, 10.0f);
		chassis_update->yaw_angle = imu_rx_data.rx_data.yaw_angle;
		
    //���µ��������ٶ� x�� ƽ���ٶ�y����ת�ٶ�wz������ϵΪ����ϵ
		chassis_update->vx = (-chassis_update->motor_chassis[0].speed + chassis_update->motor_chassis[1].speed + chassis_update->motor_chassis[2].speed - chassis_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_update->vy = (-chassis_update->motor_chassis[0].speed - chassis_update->motor_chassis[1].speed + chassis_update->motor_chassis[2].speed + chassis_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_update->wz = (-chassis_update->motor_chassis[0].speed - chassis_update->motor_chassis[1].speed - chassis_update->motor_chassis[2].speed - chassis_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
		
}

/**
  * @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
  *                 
  * @param[out]     vx_set: �����ٶ�ָ��
  * @param[out]     vy_set: �����ٶ�ָ��
  * @param[out]     chassis_rc_to_vector: "chassis" ����ָ��
  * @retval         none
  */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, fp32 *vz_set, chassis_t *chassis_rc_to_vector)
{
    if (chassis_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    
    int16_t vx_channel, vy_channel, vz_channel, vz_channel_mouse;
    fp32 vx_set_channel, vy_set_channel, vz_set_channel;

    //�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
    rc_deadband_limit(chassis_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
		rc_deadband_limit(chassis_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL], vz_channel, CHASSIS_RC_DEADLINE);
		rc_deadband_limit(chassis_rc_to_vector->chassis_RC->mouse.x, vz_channel_mouse, MOUSE_DEADBAND);//���Y��仯	
		
    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;
		if(chassis_rc_to_vector->move_mode == Home)
		{				
				vz_set_channel = vz_channel * CHASSIS_WZ_RC_SEN + vz_channel_mouse * CHASSIS_WZ_MOUSE_SEN;
		}
		else
		{
				vz_set_channel = vz_channel * CHASSIS_WZ_RC_SEN;
		}
		
		if (chassis_rc_to_vector->chassis_RC->key.W)
		{
				vx_set_channel = -NORMAL_MAX_CHASSIS_SPEED_X;
		}
		else if (chassis_rc_to_vector->chassis_RC->key.S)
		{
				vx_set_channel =  NORMAL_MAX_CHASSIS_SPEED_X;
		}
		if (chassis_rc_to_vector->chassis_RC->key.A)
		{
				vy_set_channel = -NORMAL_MAX_CHASSIS_SPEED_Y;
		}
		else if (chassis_rc_to_vector->chassis_RC->key.D)
		{
				vy_set_channel = NORMAL_MAX_CHASSIS_SPEED_Y;
		}	
		
    //һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
    first_order_filter_cali(&chassis_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);

    //ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }
		
		if(chassis_rc_to_vector->move_mode == GouDong)
		{
				if (chassis_rc_to_vector->chassis_RC->key.Q)
				{
						vz_set_channel += 0.0015f;
				}
				else if (chassis_rc_to_vector->chassis_RC->key.E)
				{
						vz_set_channel -= 0.0015f;
				}		
		}
		else
		{
				if (chassis_rc_to_vector->chassis_RC->key.Q)
				{
						vz_set_channel += 0.0035f;
				}
				else if (chassis_rc_to_vector->chassis_RC->key.E)
				{
						vz_set_channel -= 0.0035f;
				}			
		}
    *vx_set =  chassis_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = -chassis_rc_to_vector->chassis_cmd_slow_set_vy.out;
		*vz_set =  vz_set_channel;
}
/**
  * @brief          ���õ��̿�������ֵ
  * @param[out]     chassis_update:"chassis"����ָ��.
  * @retval         none
  */
static void chassis_set_contorl(chassis_t *chassis_control)
{
//		if(chassis.motor_DM_data.DM_motor_measure->motor_err == 0 && arm_message.error_info == 0)
//		{
//				chassis_control->error_info = 0;
//		}
//		else
//		{
//				chassis_control->error_info = 1;
//		}
		if(chassis_control->error_info == 1)
		{
				chassis_control->chassis_mode = NO_POWER_MODE;
		}
	
    if (chassis_control == NULL)
    {
        return;
    }
		
		fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
		chassis_rc_to_control_vector(&vx_set, &vy_set, &angle_set, chassis_control);
		//����ģʽ
		if(chassis_control->chassis_mode == NO_POWER_MODE)
		{
			chassis_control->move_mode = Home;
			chassis_control->last_move_mode = Home;
			CAN_cmd_4310_disable(DM_YAW_TX_ID,hcan2);
			DWT_Delay(0.0003f);
//			CAN_cmd_4310_disable(chassis_control->motor_DM_data[1].DM_motor_measure->motor_tx_id);
//			DWT_Delay(0.0003f);
			chassis_control->yaw_angle_set = chassis_control->yaw_angle;
			TD_set_x(&chassis_control->chassis_yaw_TD,chassis_control->yaw_angle_set);			
			chassis_control->vx_set = 0.0f;
			chassis_control->vy_set = 0.0f;
			chassis_control->wz_set = 0.0f;
		}
		//��·ģʽ||��̨��ģʽ
		else if(chassis_control->chassis_mode == RUN_MODE || chassis_control->chassis_mode == STORAGE_MODE)
		{
			switch(chassis_control->move_mode)
			{	
				case Home:
				{		
					chassis_control->yaw_angle_set = rad_format(chassis_control->yaw_angle_set + angle_set);
					break;
				}
				case Au:
				{
					chassis_control->yaw_angle_set = rad_format(chassis_control->yaw_angle_set + angle_set);					
					break;
				}
				case Ag:
				{
					chassis_control->yaw_angle_set = rad_format(chassis_control->yaw_angle_set + angle_set);
					break;
				}
				case Exchange:
				{
					chassis_control->yaw_angle_set = rad_format(chassis_control->yaw_angle_set + angle_set);
					break;
				}
				case GouDong:
				{
					chassis_control->chassis_yaw_pid.Output = 0;
					break;
				}
				default:
				{
					break;
				}
			}
			
//			if(fabs(chassis_control->yaw_angle_set - chassis_control->yaw_angle) < 0.001f)
//			{
//					chassis_control->yaw_angle_set = chassis_control->yaw_angle;
//					TD_set_x(&chassis_control->chassis_yaw_TD,chassis_control->yaw_angle_set);	
//					chassis_control->chassis_yaw_pid.Output = 0;
//			}
//			else
//			{
				TD_calc_angle(&chassis_control->chassis_yaw_TD, chassis_control->yaw_angle_set);
				PID_Calculate_Angle(&chassis_control->chassis_yaw_pid, chassis_control->yaw_angle, chassis_control->yaw_angle_set);		
//			}
			CAN_cmd_4310_enable(DM_YAW_TX_ID,hcan2);
			DWT_Delay(0.0003f);
//			CAN_cmd_4310_enable(DM_4310_M2_TX_ID);
//			DWT_Delay(0.0003f);		
			chassis_control->vx_set = vx_set;
			chassis_control->vy_set = vy_set;
			chassis_control->vx_set = fp32_constrain(chassis_control->vx_set, -SHIFT_NORMAL_MAX_CHASSIS_SPEED_X, SHIFT_NORMAL_MAX_CHASSIS_SPEED_X);
      chassis_control->vy_set = fp32_constrain(chassis_control->vy_set, -SHIFT_NORMAL_MAX_CHASSIS_SPEED_Y, SHIFT_NORMAL_MAX_CHASSIS_SPEED_Y);
			if(chassis_control->move_mode == GouDong)
			{
				chassis_control->wz_set = angle_set * 500;
			}
			else
			{
				chassis_control->wz_set = chassis_control->chassis_yaw_pid.Output;
			}
		}
		
		//�п�
		if(clamp_mode == 1)
		{
				chassis_control->motor_clamp.speed_set =  15.0f;
		}
		else if(clamp_mode ==2)
		{
				if(chassis_control->chassis_mode == STORAGE_MODE)
				{
						chassis_control->motor_clamp.position_set = -0.08f;
				}
				else
				{
						chassis_control->motor_clamp.position_set = -2.65f;
				}		
		}		
		
//		switch(chassis_control->joint_mode)
//		{
//			case GO_AHEAD_MODE: {chassis_control->motor_DM_data[0].position_set = MOTOR_AHEAD_ANGLE_SET_4310_1;
//			                     chassis_control->motor_DM_data[1].position_set = MOTOR_AHEAD_ANGLE_SET_4310_2;
//				break;
//			}
//			case GO_UP_MODE: {chassis_control->motor_DM_data[0].position_set = MOTOR_UP_ANGLE_SET_4310_1;
//				                chassis_control->motor_DM_data[1].position_set = MOTOR_UP_ANGLE_SET_4310_2;
//				break;
//			}
//			case GO_DOWN_MODE: {chassis_control->motor_DM_data[0].position_set = MOTOR_DOWN_ANGLE_SET_4310_1;
//				                  chassis_control->motor_DM_data[1].position_set = MOTOR_DOWN_ANGLE_SET_4310_2;
//				break;
//			}
//		}
		if(all_key.arm_restart_key1.itself.flag == 1 && all_key.arm_restart_key2.itself.flag == 1)
		{
				arm_restart_flag = 1;
		}					
		else{
			arm_restart_flag = 0;
		}
}

/**
  * @brief          �ĸ������ٶ���ͨ�������������������
  * @param[in]      vx_set: �����ٶ�
  * @param[in]      vy_set: �����ٶ�
  * @param[in]      wz_set: ��ת�ٶ�
  * @param[out]     wheel_speed: �ĸ������ٶ�
  * @retval         none
  */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    wheel_speed[0] = -(-vy_set - vx_set) + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = -(vy_set - vx_set) + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = -(vy_set + vx_set) + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = -(-vy_set + vx_set) + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}

/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     chassis_control_loop:"chassis"����ָ��.
  * @retval         none
  */
static void chassis_control_loop(chassis_t *chassis_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;

    //mecanum wheel speed calculation
    //�����˶��ֽ�
    chassis_vector_to_mecanum_wheel_speed(chassis_control_loop->vx_set,
                                          chassis_control_loop->vy_set, chassis_control_loop->wz_set, wheel_speed);

    //�������ӿ�������ٶȣ�������������ٶ�
    for (i = 0; i < 4; i++)
    {
        chassis_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }
    //����pid,��ֵ����
    for (i = 0; i < 4; i++)
    {
        PID_Calculate(&chassis_control_loop->chassis_motor_speed_pid[i], chassis_control_loop->motor_chassis[i].speed, chassis_control_loop->motor_chassis[i].speed_set);
				chassis_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_control_loop->chassis_motor_speed_pid[i].Output);
    }
		
		//�п�
		if(clamp_mode == 0)
		{
			chassis_control_loop->motor_clamp.give_current = 0;
		}
		else if(clamp_mode == 1)
		{
				PID_Calculate(&chassis_control_loop->clamp_motor_speed_pid, chassis_control_loop->motor_clamp.speed, chassis_control_loop->motor_clamp.speed_set);
				chassis_control_loop->motor_clamp.give_current = int16_constrain((int16_t)chassis_control_loop->clamp_motor_speed_pid.Output,-8000,8000);
		}
		else if(clamp_mode == 2)
		{		
				PID_Calculate(&chassis_control_loop->clamp_motor_position_pid, chassis_control_loop->motor_clamp.position, chassis_control_loop->motor_clamp.position_set);
				chassis_control_loop->motor_clamp.speed_set = chassis_control_loop->clamp_motor_position_pid.Output;
				PID_Calculate(&chassis_control_loop->clamp_motor_speed_pid, chassis_control_loop->motor_clamp.speed, chassis_control_loop->motor_clamp.speed_set);	
				chassis_control_loop->motor_clamp.give_current = (int16_t)(chassis_control_loop->clamp_motor_speed_pid.Output);
		}		
		
}
