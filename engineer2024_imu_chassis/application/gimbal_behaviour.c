#include "gimbal_behaviour.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "arm_math.h"
#include "laser.h"
#include "pc_task.h"
#include "TD.h"
#include "gimbal_task.h"

//遥控器死区限制
#define rc_deadband_limit(input, output, dealine)		\
{		 	                                            \
	if ((input) > (dealine) || (input) < -(dealine)) 	\
	{                                              		\
		(output) = (input);                          	\
	}                                                	\
	else                                             	\
	{                                                	\
		(output) = 0;                                	\
	}                                                	\
}
//云台是否正在反转的标志位
int8_t gimbal_turn_flag = 0;

//留意，这个底盘行为模式变量
infantry_mode_e infantry_mode = INFANTRY_MODE_RAW;
infantry_mode_e infantry_mode_last = INFANTRY_MODE_RAW;

static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_pc_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
/**
  * @brief          行为模式设置
  * @param[in]      none
  * @retval         none
  */
static void infantry_mode_set(void) {
	  //遥控器控制云台状态
    //右下表示松弛模式，电机不上电
    if (switch_is_down(gimbal_control.gimbal_remote_data->rc_ctrl_t->rc.s[GIMBAL_MODE_CHANNEL]))   	//右上小陀螺
    {
		  	infantry_mode = INFANTRY_MODE_RAW;

    }
    //右中表示编码器模式，采用底盘和云台的相对角度反馈
    else if (switch_is_mid(gimbal_control.gimbal_remote_data->rc_ctrl_t->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        infantry_mode = INFANTRY_MODE_FOLLOW_YAW;
		   //按右键或者按B进入算法通信
        if(gimbal_control.gimbal_remote_data->rc_ctrl_t->mouse.press_r)
        {
            infantry_mode = INFANTRY_MODE_PC;
        }
    }
    //右上采用陀螺仪绝对角度控制
    else if (switch_is_up(gimbal_control.gimbal_remote_data->rc_ctrl_t->rc.s[GIMBAL_MODE_CHANNEL]))
    {
            infantry_mode = INFANTRY_MODE_PC;
    }
       infantry_mode_last = infantry_mode; //记录上一次模式
	}
void gimbal_behaviour_mode_set()
{

    //步兵行为状态设置
    infantry_mode_set();
    //根据云台行为状态机设置电机状态
    //云台松弛模式(无力)
    if (infantry_mode == INFANTRY_MODE_RAW)
    {
        gimbal_control.gimbal_mode= GIMBAL_MODE_ZERO_FORCE;
    }
    //陀螺仪模式，用陀螺仪反馈
    else if (infantry_mode == INFANTRY_MODE_FOLLOW_YAW || infantry_mode == INFANTRY_MODE_NO_FOLLOW_YAW \
			||infantry_mode == INFANTRY_MODE_GYRO)
    {
        gimbal_control.gimbal_mode= GIMBAL_MODE_ABSOLUTE_ANGLE;
    }

    //PC辅助瞄准
    else if(infantry_mode == INFANTRY_MODE_PC)
    {
        gimbal_control.gimbal_mode= GIMBAL_MODE_PC;
    }
		
}

//云台行为控制
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch)
{
    if (add_yaw == NULL || add_pitch == NULL)
    {
        return;
    }
    if ( gimbal_control.gimbal_mode == GIMBAL_MODE_ZERO_FORCE)
    {
        gimbal_zero_force_control(add_yaw, add_pitch, &gimbal_control);
    }
    else if (gimbal_control.gimbal_mode  == GIMBAL_MODE_ABSOLUTE_ANGLE)
    {
        gimbal_absolute_angle_control(add_yaw, add_pitch, &gimbal_control);
    }
    else if ( gimbal_control.gimbal_mode  == GIMBAL_MODE_PC)
    {
        gimbal_pc_angle_control(add_yaw, add_pitch, &gimbal_control);
    }
}

//无力模式控制
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}

//陀螺仪模式控制
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static int16_t yaw_channel = 0, pitch_channel = 0;
    static float yaw_channel_mouse = 0, pitch_channel_mouse = 0;
 //   static float last_yaw_channel_mouse = 0, last_pitch_channel_mouse = 0;
    //遥控器死区控制
    rc_deadband_limit(gimbal_control_set->gimbal_remote_data->rc_ctrl_t->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);//Y 轴
    rc_deadband_limit(gimbal_control_set->gimbal_remote_data->rc_ctrl_t->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);//P轴
    rc_deadband_limit(gimbal_control_set->gimbal_remote_data->rc_ctrl_t->mouse.x, yaw_channel_mouse, MOUSE_DEADBAND);//鼠标X轴变化
    rc_deadband_limit(gimbal_control_set->gimbal_remote_data->rc_ctrl_t->mouse.y, pitch_channel_mouse, MOUSE_DEADBAND);//鼠标Y轴变化
     *yaw = yaw_channel * YAW_RC_SEN + yaw_channel_mouse * YAW_MOUSE_SMALLSEN;
     *pitch = pitch_channel * PITCH_RC_SEN + pitch_channel_mouse * YAW_MOUSE_SMALLSEN;
    static uint16_t last_turn_keyboard = 0;//上一次按键
    static fp32 gimbal_end_angle = 0.0f;
    //掉头功能（X键）
    if ((gimbal_control_set->gimbal_remote_data->rc_ctrl_t->key.v & TURN_KEYBOARD) && !(last_turn_keyboard & TURN_KEYBOARD))//防止连击误触
    {
        if (gimbal_turn_flag == 0)
        {
            gimbal_turn_flag = 1;
            //保存掉头的目标值
            gimbal_end_angle = rad_format(gimbal_control_set->gimbal_yaw_motor.absolute_angle + PI);
        }
    }
    last_turn_keyboard = gimbal_control_set->gimbal_remote_data->rc_ctrl_t->key.v;
    if (gimbal_turn_flag)
    {
        //控制掉头的目标值
        if (rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle) > 0.0f)//弧度格式化为-PI~PI
        {
            *yaw += TURN_SPEED;//每次改变的角度
        }
        else
        {
            *yaw -= TURN_SPEED;
        }
    }
    //转到PI后停止
    if (gimbal_turn_flag && fabs(rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle)) < 0.01f)
    {
        gimbal_turn_flag = 0;
    }
}

//PC模式控制
static void gimbal_pc_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
	if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
	{
		return;
    }
	static float yaw_channel = 0, pitch_channel = 0 ;

	yaw_channel = pc_receive_msg.rx_data.shoot_yaw;
	pitch_channel = pc_receive_msg.rx_data.shoot_pitch;

//	static const fp32 pc_fliter_num[3] = {1.822751349748f, -0.8371810677374f,  0.01442971798952f};
//	static fp32 pitch_fliter[3] = {0.0f, 0.0f, 0.0f};
//	static fp32 yaw_fliter[3] = {0.0f, 0.0f, 0.0f};
//	pitch_fliter[0] = pitch_fliter[1];
//	pitch_fliter[1] = pitch_fliter[2];
//	pitch_fliter[2] = pitch_fliter[1] * pc_fliter_num[0] + pitch_fliter[0] * pc_fliter_num[1] +  pitch_channel * pc_fliter_num[2];

//	yaw_fliter[0] = yaw_fliter[1];
//	yaw_fliter[1] = yaw_fliter[2];
		
//	yaw_fliter[2] = yaw_fliter[1] * pc_fliter_num[0] + yaw_fliter[0] * pc_fliter_num[1] +  yaw_channel * pc_fliter_num[2];
//	*yaw = yaw_fliter[2] * YAW_PC_SEN; 
//	*pitch = pitch_fliter[2] * PITCH_PC_SEN;	
	TD_calc(&gimbal_control_set->gimbal_yaw_motor.PC_Control_Td,yaw_channel * YAW_PC_SEN);
	TD_calc(&gimbal_control_set->gimbal_pitch_motor.PC_Control_Td,pitch_channel * PITCH_PC_SEN);
	if(fabs(gimbal_control_set->gimbal_yaw_motor.absolute_angle *  pc_receive_msg.rx_data.shoot_yaw) > 9.3f){
		gimbal_control_set->gimbal_yaw_motor.PC_Control_Td.x = pc_receive_msg.rx_data.shoot_yaw;
	}
	*yaw = gimbal_control_set->gimbal_yaw_motor.PC_Control_Td.x;
	*pitch = gimbal_control_set->gimbal_pitch_motor.PC_Control_Td.x;	
//	*yaw = yaw_channel;
//	*pitch = pitch_channel;	
}

//若处于无力模式则停止射击
bool_t gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal_control.gimbal_mode == GIMBAL_MODE_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
