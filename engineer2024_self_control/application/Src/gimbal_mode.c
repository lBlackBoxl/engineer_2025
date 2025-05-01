#include "gimbal_mode.h"
#include "can_solve.h"
#include "pc_auto_aim.h"
#include "MATH_LIB.h"
#include "pc_speed.h"
#include "fuzzy_pid.h"
#include "feedforward.h"
#include "main.h"
#include "TD.h"
//遥控器死区限制
void rc_deadband_limit(int16_t input, int16_t *output, int16_t dealine)		
{		 	                                            
	if ((input) > (dealine) || (input) < -(dealine)) 	
	{                                              		
		(*output) = (input);                          	
	}                                                	
	else                                             	
	{                                                	
		(*output) = 0;                                	
	}                                                	
}
uint8_t first_time;
//函数声明
void motor_mode_set(gimbal_control_t *motor_set);
void gimbal_angle_add(float32_t *add_yaw, float32_t *add_pitch, gimbal_control_t *gimbal_control_set);
void gimbal_zero_force_control(float32_t *yaw, float32_t *pitch, gimbal_control_t *gimbal_control_set);
void gimbal_absolute_angle_control(float32_t *yaw, float32_t *pitch, gimbal_control_t *gimbal_control_set);
void gimbal_pc_control_fire(float32_t *yaw, float32_t *pitch, gimbal_control_t *gimbal_control_set);
void gimbal_pc_control_normal(float32_t *yaw, float32_t *pitch, gimbal_control_t *gimbal_control_set);
void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, float32_t add);
void gimbal_relative_angle_control(float32_t *yaw, float32_t *pitch, gimbal_control_t *gimbal_control_set);
void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, float32_t add);
void motor_zero_force_control(gimbal_motor_t *gimbal_motor);
void motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);
void motor_relative_angle_control(gimbal_motor_t *gimbal_motor);
//自瞄滤波
static const float32_t pc_fliter_num[3] = {1.822751349748f, -0.8371810677374f,  0.01442971798952f};
static float32_t pitch_fliter[3] = { 1.0f, -1.397499543939001220849149831337854266167f, 0.536983400208190464475421777024166658521f};
static float32_t yaw_fliter[3] = {1.0f, -1.397499543939001220849149831337854266167f, 0.536983400208190464475421777024166658521f};
//电机模式设置
void motor_mode_set(gimbal_control_t *motor_set)
{
    if(motor_set->gimbal_mode == GIMBAL_ZERO_FORCE)
    {
        motor_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ZORE;
        motor_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ZORE;
    }
    else if(motor_set->gimbal_mode == GIMBAL_RC_CONTROL || motor_set->gimbal_mode == GIMBAL_PC_CONTROL)
    {   
		motor_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        motor_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
}
//计算角度变化量，两种情况，遥控器给的是增量，PC给的是要达到的设定值，就不算add了
void gimbal_angle_add(float32_t *add_yaw, float32_t *add_pitch, gimbal_control_t *gimbal_control_set)
{
    if(add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL) return;
    if(gimbal_control_set->gimbal_mode == GIMBAL_ZERO_FORCE)
        gimbal_zero_force_control(add_yaw, add_pitch, gimbal_control_set);
    else if(gimbal_control_set->gimbal_mode == GIMBAL_RC_CONTROL)
    {
		first_time = 0;
		//自瞄控制权
		#if RC_DEBUG
		gimbal_pc_control_fire(add_yaw, add_pitch, gimbal_control_set);
		//遥控器控制
		#else
			gimbal_absolute_angle_control(add_yaw, add_pitch, gimbal_control_set);
		#endif

	}
	else if(gimbal_control_set->gimbal_mode == GIMBAL_PC_CONTROL || gimbal_control_set->gimbal_mode == GIMBAL_PC_AUTO_AIM || gimbal_control_set->gimbal_mode == GIMBAL_PC_AUTO_RUN)
	{
//		if(auto_aim_send_msg.tx_data.state == '0')
//		{
//			if(gimbal_control_set->gimbal_last_mode != GIMBAL_PC_AUTO_AIM && gimbal_control_set->gimbal_last_mode == GIMBAL_PC_AUTO_AIM)
//			{
//				TD_init(&gimbal_control_set->gimbal_yaw_motor.PC_ANGLE_TD, 50, 2, 0.001, gimbal_control_set->gimbal_yaw_motor.absolute_angle);
//				TD_init(&gimbal_control_set->gimbal_pitch_motor.PC_ANGLE_TD, 50, 2, 0.001, gimbal_control_set->gimbal_pitch_motor.absolute_angle);
//			}
			gimbal_pc_control_fire(add_yaw, add_pitch, gimbal_control_set);
			gimbal_control_set->gimbal_mode = GIMBAL_PC_AUTO_AIM;
			gimbal_control_set->gimbal_last_mode = GIMBAL_PC_AUTO_AIM;
//		}
//		else if(auto_aim_send_msg.tx_data.state == '1')
//		{
//			first_time = 0;
//			gimbal_pc_control_normal(add_yaw, add_pitch, gimbal_control_set);
//			gimbal_control_set->gimbal_mode = GIMBAL_PC_AUTO_RUN;
//			gimbal_control_set->gimbal_last_mode = GIMBAL_PC_AUTO_RUN;
//		}
			
	}
}
//无力模式
void gimbal_zero_force_control(float32_t *yaw, float32_t *pitch, gimbal_control_t *gimbal_control_set)
{
	if(yaw == NULL || pitch == NULL || gimbal_control_set == NULL) return;
	*yaw = 0;
	*pitch = 0;
}
//(遥控器)

void gimbal_absolute_angle_control(float32_t *yaw, float32_t *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL) return;
    static int16_t yaw_channel = 0, pitch_channel = 0;
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], &yaw_channel, RC_DEADBAND);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], &pitch_channel, RC_DEADBAND);
	*yaw = yaw_channel * YAW_RC_SEN;
    *pitch = pitch_channel * PITCH_RC_SEN;
}
//路径规划控制，pitch归中，yaw为速度环控制，没有角度环
uint8_t pitch_flag = 0;
float32_t yaw_angle_set;
void gimbal_pc_control_normal(float32_t *yaw, float32_t *pitch, gimbal_control_t *gimbal_control_set)
{
	*yaw = 0.0f;
	*pitch = 0.0f;
	gimbal_control_set->gimbal_pitch_motor.absolute_angle_set = gimbal_control_set->gimbal_pitch_motor.absolute_angle_offset;
	gimbal_control_set->gimbal_yaw_motor.absolute_angle_set = gimbal_control_set->gimbal_yaw_motor.absolute_angle;
//	yaw_angle_set = gimbal_control_set->gimbal_yaw_motor.absolute_angle;
	gimbal_control_set->gimbal_yaw_motor.motor_gyro_set = vel_ctrl.vw;


}
//视觉控制云台

float32_t curr_abs_angle;
int speed_deriction;
float32_t pitch_angle_set;
void gimbal_pc_control_fire(float32_t *yaw, float32_t *pitch, gimbal_control_t *gimbal_control_set)
{
//	if(auto_aim_receive_msg.rx_data.turn == 1)
//	{
//		gimbal_control_set->gimbal_yaw_motor.motor_gyro_set = -0.7f;
//		if(first_time == 0)
//		{
//			TD_init(&gimbal_control_set->gimbal_pitch_motor.PC_ANGLE_TD, 50, 2, 0.001, gimbal_control_set->gimbal_pitch_motor.absolute_angle);
//			pitch_angle_set = gimbal_control_set->gimbal_pitch_motor.max_absolute_angle - 0.15f;
//			first_time = 1;
//		}
//		if(fabs(gimbal_control_set->gimbal_pitch_motor.absolute_angle -  gimbal_control_set->gimbal_pitch_motor.min_absolute_angle) < 0.1f)
//		{
//			pitch_angle_set = gimbal_control_set->gimbal_pitch_motor.max_absolute_angle - 0.15f;
//		}
//		if(fabs(gimbal_control_set->gimbal_pitch_motor.absolute_angle -  gimbal_control_set->gimbal_pitch_motor.max_absolute_angle + 0.15f) < 0.1f)
//		{
//			pitch_angle_set = gimbal_control_set->gimbal_pitch_motor.min_absolute_angle;
//		}
//		TD_calc(&gimbal_control_set->gimbal_pitch_motor.PC_ANGLE_TD, pitch_angle_set);
//		gimbal_control_set->gimbal_pitch_motor.absolute_angle_set = gimbal_control_set->gimbal_pitch_motor.PC_ANGLE_TD.x;
//		//重新初始化角度环，使pitch角速度不会过快
//		gimbal_control_set->gimbal_pitch_motor.absolute_angle_set = -0.1f;
//		gimbal_PID_init(&gimbal_control_set->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, 2.0f, 0.2f, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
//		PID_init(&gimbal_control_set->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, 10000, 10, 50, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);
//	}
//	else
//	{
		//初始化pitch角度环pid，加快跟随
		first_time = 0;
		*yaw = 0.0f;
		*pitch = 0.0f;
		gimbal_PID_init(&gimbal_control_set->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
		PID_init(&gimbal_control_set->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);
		
		gimbal_PID_init(&gimbal_control_set->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
		PID_init(&gimbal_control_set->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
		
//		TD_calc(&gimbal_control_set->gimbal_yaw_motor.PC_ANGLE_TD, auto_aim_receive_msg.rx_data.shoot_yaw);
//		TD_calc(&gimbal_control_set->gimbal_pitch_motor.PC_ANGLE_TD, auto_aim_receive_msg.rx_data.shoot_pitch);
//		gimbal_control_set->gimbal_yaw_motor.absolute_angle_set = gimbal_control_set->gimbal_yaw_motor.PC_ANGLE_TD.x;
//		gimbal_control_set->gimbal_pitch_motor.absolute_angle_set = gimbal_control_set->gimbal_pitch_motor.PC_ANGLE_TD.x;
		
		
		pitch_fliter[0] = pitch_fliter[1];
		pitch_fliter[1] = pitch_fliter[2];
		pitch_fliter[2] = pitch_fliter[1] * pc_fliter_num[0] + pitch_fliter[0] * pc_fliter_num[1] +  auto_aim_receive_msg.rx_data.shoot_pitch * pc_fliter_num[2];
		yaw_fliter[0] = yaw_fliter[1];
		yaw_fliter[1] = yaw_fliter[2];
		yaw_fliter[2] = yaw_fliter[1] * pc_fliter_num[0] + yaw_fliter[0] * pc_fliter_num[1] +  auto_aim_receive_msg.rx_data.shoot_yaw * pc_fliter_num[2];
		gimbal_control_set->gimbal_yaw_motor.absolute_angle_set = yaw_fliter[2];
		gimbal_control_set->gimbal_pitch_motor.absolute_angle_set = pitch_fliter[2];
		gimbal_control_set->gimbal_yaw_motor.absolute_angle_set = auto_aim_receive_msg.rx_data.shoot_yaw ;//* 0.5f + gimbal_control_set->gimbal_yaw_motor.absolute_angle * 0.5f;
		gimbal_control_set->gimbal_pitch_motor.absolute_angle_set = auto_aim_receive_msg.rx_data.shoot_pitch ;//* 0.5f + gimbal_control_set->gimbal_pitch_motor.absolute_angle * 0.5f;
		gimbal_control_set->gimbal_pitch_motor.absolute_angle_set = float32_limit(gimbal_control_set->gimbal_pitch_motor.absolute_angle_set, gimbal_control_set->gimbal_pitch_motor.min_absolute_angle, gimbal_control_set->gimbal_pitch_motor.max_absolute_angle);
//	}
}
//绝对角度控制(控制范围)
void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, float32_t add)
{
	if(gimbal_control.gimbal_mode == GIMBAL_PC_AUTO_AIM) return;
	static float32_t bias_angle;
	static float32_t angle_set;
	if(gimbal_motor == NULL) return;
	//当前控制误差（理想情况下为0）
	bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
	if(gimbal_motor == &gimbal_control.gimbal_pitch_motor)
	{
		
		if(gimbal_motor->absolute_angle + bias_angle + add > gimbal_motor->max_absolute_angle)
		{
			//正方向最大机械角度
			if(add > 0.0f) add = gimbal_motor->max_absolute_angle - gimbal_motor->absolute_angle - bias_angle;
		}
		else if(gimbal_motor->absolute_angle + bias_angle +add < gimbal_motor->min_absolute_angle)
		{
			//负方向最大机械角度
			if(add < 0.0f) add = gimbal_motor->min_absolute_angle -  gimbal_motor->absolute_angle - bias_angle;
		}
//		if(gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
//		{
//			//正方向最大机械角度
//			if(add > 0.0f) add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
//		}
//		else if(gimbal_motor->relative_angle + bias_angle +add < gimbal_motor->min_relative_angle)
//		{
//			//负方向最大机械角度
//			if(add < 0.0f) add = gimbal_motor->min_relative_angle -  gimbal_motor->relative_angle - bias_angle;
//		}
	}
	angle_set = gimbal_motor->absolute_angle_set;
	gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}
//编码器、相对角度模式控制，云台跟随底盘
void gimbal_relative_angle_control(float32_t *yaw, float32_t *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL) return;
    static int16_t pitch_channel = 0;
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], &pitch_channel, RC_DEADBAND);
    *yaw = 0.0f;
    *pitch =  pitch_channel * PITCH_RC_SEN;
}
//相对角度控制(控制范围)
void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, float32_t add)
{
	if(gimbal_motor == NULL) return;
	gimbal_motor->relative_angle_set += add;
	if(gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
		gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
	else if(gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
		gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
}
//电机无力模式下发送电流
void motor_zero_force_control(gimbal_motor_t *gimbal_motor)
{
	if(gimbal_motor == NULL) return;
	gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
	gimbal_motor->given_current = gimbal_motor->current_set;
}
//绝对角度计算电流
void motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
	float32_t yaw_gyro_feedforward_set;
	float32_t pitch_gyro_feedforward_set;
	if(gimbal_motor == NULL) return;
	#if ADRC_MODE
		if(gimbal_motor == &gimbal_control.gimbal_yaw_motor)
		{
			if(gimbal_control.gimbal_mode == GIMBAL_PC_AUTO_AIM || gimbal_control.gimbal_mode == GIMBAL_PC_AUTO_RUN)
			{
				//当前视觉控制
				if(gimbal_control.gimbal_mode == GIMBAL_PC_AUTO_AIM)
				{
//					if(auto_aim_receive_msg.rx_data.turn == 0)
//					{
						//自瞄
						gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
//						gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
//						gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
//					}
//					else if(auto_aim_receive_msg.rx_data.turn == 1)
//					{
//						gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_PC_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
//						gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
//					}
					
					gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_PC_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
					gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
					
				}
				//当前路径规划控制
				else if(gimbal_control.gimbal_mode == GIMBAL_PC_AUTO_RUN)
				{
					gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_PC_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
					gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
				}
			}
			else if(gimbal_control.gimbal_mode == GIMBAL_RC_CONTROL)
			{	
	     		gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
//				yaw_gyro_feedforward_set = Feedforward_Calculate(&gimbal_motor->motor_angle_feedforward,gimbal_motor->absolute_angle_set);
				gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
				gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
			}
			else if(gimbal_control.gimbal_mode == GIMBAL_ZERO_FORCE)
			{
				gimbal_motor->given_current = 0.0f;
			}
		}
		else if(gimbal_motor == &gimbal_control.gimbal_pitch_motor)
		{	
//			pitch_gyro_feedforward_set = Feedforward_Calculate(&gimbal_motor->motor_angle_feedforward,gimbal_motor->absolute_angle_set);
			gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
			gimbal_motor->given_current = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
			if(gimbal_control.gimbal_mode == GIMBAL_ZERO_FORCE)
			{
				gimbal_motor->given_current = 0.0f;
			}
		}
	#else
		//角度环、速度环串级pid计算
		gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
		gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
		//pitch轴需要float
		if(gimbal_motor == &gimbal_control.gimbal_pitch_motor)
			gimbal_motor->given_current = (float32_t)(gimbal_motor->current_set);
		else if(gimbal_motor == &gimbal_control.gimbal_yaw_motor)
			gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
	#endif
}
//相对角度计算电流
void motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
	if(gimbal_motor == NULL) return;
	gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, gimbal_motor->motor_gyro);
	gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
	gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}
