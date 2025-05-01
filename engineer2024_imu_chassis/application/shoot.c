#include "shoot.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "arm_math.h"
#include "gimbal_behaviour.h"
#include "laser.h"
#include "remote_control.h"
#include "gimbal_task.h"
#define shoot_laser_on() laser_on()//激光开启宏定义
#define shoot_laser_off() laser_off()//激光关闭宏定义
//extern uint8_t bullet_speed_corret_flag;
shoot_control_t shoot_control;//射击数据
uint8_t shoot_fire_state;					// 摩擦轮开启 UI
uint8_t block_flag = 0;					// 拨弹盘堵转 标志位、UI
//发射模式切换

uint8_t continue_shoot_start_flag;
uint8_t continue_shoot_end_flag;
static  int8_t last_s = RC_SW_UP;
ramp_function_source_t fric_ramp;
uint8_t READY_Flag;
slid_avg_filter_t fire_speed_avg_filter;
static void shoot_feedback_update(void);
static void shoot_set_mode(void);
static void trigger_block_check(void);
static void shoot_bullet_control(void);
static void shoot_adjust_control(void);
//static void shoot_heart_control_low(void);
static void shoot_heart_control_high(void);
//static void cali_shoot_delay_time(void);
int fire_heat_flag = 0;
//发射机构初始化
void shoot_init(void)
{
    //PID参数初始化
    // static const fp32 Trigger_speed_pid[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};
    static const fp32 Trigger_angle_pid[3] = {TRIGGER_ANGLE_PID_KP,TRIGGER_ANGLE_PID_KI,TRIGGER_ANGLE_PID_KD};
    //static const fp32 fric_speed_pid[3] = {FRIC_SPEED_PID_KP, FRIC_SPEED_PID_KI, FRIC_SPEED_PID_KD};
    //发射模式初始化
    shoot_control.shoot_mode = SHOOT_STOP;
    //遥控器指针
    shoot_control.shoot_remote_data = get_remote_data_point();
    shoot_control.shoot_rc = get_remote_control_point();
    //开火标志位
    shoot_control.shoot_pc_data.pc_rx_data = get_pc_rx_point();
    //电机指针
    shoot_control.trigger_motor_measure = get_trigger_motor_measure_point();
    shoot_control.fric1_motor_measure = get_fric1_motor_measure_point();
    shoot_control.fric2_motor_measure = get_fric2_motor_measure_point();
    //初始化PID
    // PID_init(&shoot_control.trigger_speed_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_SPEED_PID_MAX_OUT, TRIGGER_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_control.trigger_angle_pid, PID_POSITION, Trigger_angle_pid, TRIGGER_ANGLE_PID_MAX_OUT, TRIGGER_ANGLE_PID_MAX_IOUT);
    //  PID_init(&shoot_control.fric_motor_pid, PID_POSITION, fric_speed_pid, FRIC_SPEED_PID_MAX_OUT, FRIC_SPEED_PID_MAX_OUT);

    PID_Init(&shoot_control.trigger_speed_pid,TRIGGER_SPEED_PID_MAX_OUT,TRIGGER_SPEED_PID_MAX_IOUT,0.0f,TRIGGER_SPEED_PID_KP,TRIGGER_SPEED_PID_KI,TRIGGER_SPEED_PID_KD,0.0f,0.0f,0.00159154943091895335768883763373f,0.0f,4,0x11);
    //  PID_Init(&shoot_control.trigger_angle_pid,TRIGGER_ANGLE_PID_MAX_OUT,TRIGGER_ANGLE_PID_MAX_IOUT,0.0f,TRIGGER_ANGLE_PID_KP,TRIGGER_ANGLE_PID_KI,TRIGGER_ANGLE_PID_KD,0.0f,0.0f,0.00159154943091895335768883763373f,0.0f,4,0x11);
    PID_Init(&shoot_control.fric_motor_pid[0],FRIC_SPEED_PID_MAX_OUT,FRIC_SPEED_PID_MAX_IOUT,0.0f,FRIC_SPEED_PID_KP,FRIC_SPEED_PID_KI,FRIC_SPEED_PID_KD,0.0f,0.0f,0.00159154943091895335768883763373f,0.0f,4,0x11);
    PID_Init(&shoot_control.fric_motor_pid[1],FRIC_SPEED_PID_MAX_OUT,FRIC_SPEED_PID_MAX_IOUT,0.0f,FRIC_SPEED_PID_KP,FRIC_SPEED_PID_KI,FRIC_SPEED_PID_KD,0.0f,0.0f,0.00159154943091895335768883763373f,0.0f,4,0x11);
    //更新数据
    shoot_control.bullet_speed_set = BULLET_SPEED_SET;
    shoot_control.bullet_speed_fix = BULLET_SPEED_FIX_K;
    TD_init(&shoot_control.fric_speed_set_td,10,20,0.001,0);    //初始化TD
    shoot_feedback_update();
    shoot_control.fric_speed_set = 0;
    shoot_control.trigger_angle = 0.0f;
    shoot_control.trigger_angle_set = 0.0f;
    READY_Flag = 0;
    block_flag = 0;
}

/**
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */
int16_t shoot_control_loop(void)
{

    shoot_set_mode();        //设置状态机
    shoot_feedback_update(); //更新数据
    if(shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_laser_off();  //关闭激光
        //设置拨弹轮的速度
        shoot_control.trigger_speed_set = 0.0f;
        shoot_control.trigger_angle_set = shoot_control.trigger_angle;

        shoot_control.trigger_given_current = 0;
        shoot_control.fric_speedset[0] = 0.0f;
        shoot_control.fric_speedset[1] = 0.0f;
        shoot_fire_state = 0;
    }
    else
    {
        if( shoot_control.shoot_pc_data.pc_rx_data->fire == 1 && gimbal_control.gimbal_mode == GIMBAL_MODE_PC) {
            shoot_control.shoot_pc_data.pc_continue_flag = 1;
            shoot_control.shoot_pc_data.pc_continue_count = 0;
        }
        if (shoot_control.shoot_pc_data.pc_continue_flag == 1 && shoot_control.shoot_pc_data.pc_continue_count < PC_TIME)
        {
            shoot_control.shoot_pc_data.pc_continue_count ++;
        }
        shoot_fire_state = 1;
        shoot_laser_on(); //激光开启


        if (shoot_control.shoot_mode == SHOOT_READY)
        {
        //是否按下G键
        if (shoot_control.shoot_remote_data->key_g.key_switch == 0)
        {   //G没按下
						shoot_control.fric_speed_set = SHOOT_FIRE_SPEED_LOW;
        }
        else//G按下了
        {   //没有堵转并且拨码开关在下 或者 处于自瞄模式并开火 或者 按着鼠标左键
						shoot_control.fric_speed_set = SHOOT_FIRE_SPEED_HIGH;
        }						
            

//					if(READY_Flag == 0){
//						shoot_control.shoot_mode = SHOOT_ADJUST;
//						READY_Flag++;
//					}

            shoot_control.trigger_speed_set = shoot_PID_calc(&shoot_control.trigger_angle_pid,shoot_control.trigger_angle,shoot_control.trigger_angle_set);
        }
        else if (shoot_control.shoot_mode == SHOOT_BULLET)//单发
        {
            shoot_bullet_control();
        }
        else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)//连发
        {
            TD_calc(&shoot_control.fric_speed_set_td, shoot_control.bullet_speed_fix * shoot_control.bullet_speed_delta);
            shoot_control.fric_speed_set = SHOOT_FIRE_SPEED_LOW + shoot_control.fric_speed_set_td.x;
            //热量控制
//            if(gimbal_control.gimbal_remote_data->key_v.key_switch == 1) { //高速
                shoot_heart_control_high();
//            } else { //低速
//                shoot_heart_control_low();
//            }

            /* 还在连发就收到下一次的连发指令，计数清零*/
            if( pc_receive_msg.rx_data.fire  == 1 && shoot_control.shoot_pc_data.pc_continue_flag == 1 && gimbal_control.gimbal_mode == GIMBAL_MODE_PC)//pc_receive_msg.rx_data.fire
            {
                shoot_control.shoot_pc_data.pc_continue_count = 0;
            }
        }
        else if(shoot_control.shoot_mode == SHOOT_DONE)//射击完成
        {
            shoot_control.trigger_speed_set = 0.0f;
        }
        else if(shoot_control.shoot_mode == SHOOT_ADJUST)
        {
            shoot_control.trigger_speed_set =  SHOOT_ADJUST_SPEED;//反转至卡弹位置
            shoot_adjust_control();
        }

        trigger_block_check();
        //计算拨弹轮电机PID
        //引入积分分离以减小超调
        if(fabs(shoot_control.trigger_speed_pid.Err) >= 0.3f)
            shoot_control.trigger_speed_pid.Iout = 0;
        PID_Calculate(&shoot_control.trigger_speed_pid, shoot_control.trigger_speed, shoot_control.trigger_speed_set);
        shoot_control.trigger_given_current = (int16_t)(shoot_control.trigger_speed_pid.Output);
        shoot_control.fric_speedset[0] = - shoot_control.fric_speed_set;
        shoot_control.fric_speedset[1] =  shoot_control.fric_speed_set;
    }
    shoot_control.fric_given_current[0] = PID_Calculate(&shoot_control.fric_motor_pid[0], shoot_control.fric_speed[0], shoot_control.fric_speedset[0]);
    shoot_control.fric_given_current[1] = PID_Calculate(&shoot_control.fric_motor_pid[1], shoot_control.fric_speed[1], shoot_control.fric_speedset[1]);

    return shoot_control.trigger_given_current;
}


//发射模式切换
static void shoot_set_mode(void)
{
    if(shoot_control.shoot_mode == SHOOT_STOP) {
        //上拨判断， 拨一次开启摩擦轮
        if (switch_is_up(shoot_control.shoot_remote_data->rc_ctrl_t->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s)) {
            shoot_control.shoot_mode = SHOOT_READY_FRIC;
        }
        if(switch_is_mid(shoot_control.shoot_remote_data->rc_ctrl_t->rc.s[SHOOT_RC_MODE_CHANNEL])\
                &&(shoot_control.shoot_remote_data->rc_ctrl_t->key.v & SHOOT_ON_KEYBOARD)) {
            shoot_control.shoot_mode = SHOOT_READY_FRIC;
        }
    } else {
        if (switch_is_up(shoot_control.shoot_remote_data->rc_ctrl_t->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s)) {
            shoot_control.shoot_mode = SHOOT_STOP;
        }
        if (switch_is_mid(shoot_control.shoot_remote_data->rc_ctrl_t->rc.s[SHOOT_RC_MODE_CHANNEL]) \
                &&(shoot_control.shoot_remote_data->rc_ctrl_t->key.v & SHOOT_OFF_KEYBOARD)) {
            shoot_control.shoot_mode = SHOOT_STOP;
        }
    }
    //如果摩擦轮准备模式并且摩擦轮达到设定速度
    if(shoot_control.shoot_mode == SHOOT_READY_FRIC)
    {
        if(fabs(shoot_control.fric_speed[0]-shoot_control.fric_speedset[0])<FRIC_DEADBAND\
                && fabs(shoot_control.fric_speed[1]-shoot_control.fric_speedset[1])<FRIC_DEADBAND) {
            shoot_control.shoot_mode = SHOOT_READY;//进入准备模式
        }
    }
    else if(shoot_control.shoot_mode == SHOOT_READY)
    {
        if (switch_is_down(shoot_control.shoot_remote_data->rc_ctrl_t->rc.s[SHOOT_RC_MODE_CHANNEL])&& !switch_is_down(last_s)) {
            shoot_control.shoot_mode = SHOOT_BULLET;
        }
        //是否按下G键
        if (shoot_control.shoot_remote_data->key_g.key_switch == 0)
        {   //G没按下
            //拨码开关第一次向下 或者 处于自瞄模式并开火 或者 左击一次鼠标
            if(gimbal_control.gimbal_mode == GIMBAL_MODE_PC && shoot_control.shoot_pc_data.pc_rx_data->fire  == 1) {
                shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
            }
            if(shoot_control.shoot_remote_data->mouse_press_l.press == 1 ) {
                shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
            }
        }
        else//G按下了
        {   //没有堵转并且拨码开关在下 或者 处于自瞄模式并开火 或者 按着鼠标左键
            if(gimbal_control.gimbal_mode == GIMBAL_MODE_PC && shoot_control.shoot_pc_data.pc_rx_data->fire  == 1) {
                shoot_control.shoot_mode = SHOOT_BULLET;
            }
            if(shoot_control.shoot_remote_data->mouse_press_l.press == 1 && shoot_control.shoot_remote_data->mouse_press_l.last_press == 0 ) {
                shoot_control.shoot_mode = SHOOT_BULLET;
            }
        }
        //单发时间超过阈值
        if (shoot_control.rc_s_time >= RC_S_LONG_TIME) {
            shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
        }
        /*pc控制发弹*/
        if(shoot_control.shoot_pc_data.pc_continue_flag == 1 && gimbal_control.gimbal_mode == GIMBAL_MODE_PC) {
            shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
        }
    }
    else if(shoot_control.shoot_mode == SHOOT_BULLET) {
        //单发并且拨码不在下
        if(!switch_is_down(shoot_control.shoot_remote_data->rc_ctrl_t->rc.s[SHOOT_RC_MODE_CHANNEL])&& switch_is_down(last_s)) {
            shoot_control.shoot_mode = SHOOT_DONE;
        }
        if(shoot_control.shoot_remote_data->mouse_press_l.press == 0 && shoot_control.shoot_remote_data->key_g.key_switch == 1) {
            shoot_control.shoot_mode = SHOOT_DONE;
        }
    }//拨杆从下到中或者pc连发一小段时间（保证连发射出子弹）后停止连发
    else if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET ) {
        if(!switch_is_down(shoot_control.shoot_remote_data->rc_ctrl_t->rc.s[SHOOT_RC_MODE_CHANNEL]) && switch_is_down(last_s)) {
            shoot_control.shoot_mode = SHOOT_ADJUST;
        }
//				if(gimbal_control.gimbal_mode == GIMBAL_MODE_PC&&shoot_control.shoot_pc_data.pc_rx_data->fire == 0){
//						shoot_control.shoot_mode = SHOOT_DONE;
//						shoot_control.trigger_angle_set = shoot_control.trigger_angle - PI_FOUR;
//				}
				if(gimbal_control.gimbal_mode != GIMBAL_MODE_PC && shoot_control.shoot_remote_data->mouse_press_l.press == 0 \
                && shoot_control.shoot_remote_data->key_g.key_switch == 0 \
				        && switch_is_mid(shoot_control.shoot_remote_data->rc_ctrl_t->rc.s[SHOOT_RC_MODE_CHANNEL])){
						shoot_control.shoot_mode = SHOOT_DONE;
						shoot_control.trigger_angle_set = shoot_control.trigger_angle - PI_FOUR;
								}
				if(shoot_control.shoot_pc_data.pc_continue_count == PC_TIME) {
            shoot_control.shoot_mode = SHOOT_DONE;
					  shoot_control.trigger_angle_set = shoot_control.trigger_angle - PI_FOUR;
        }
    }
    else if(shoot_control.shoot_mode == SHOOT_DONE) //射击完成
    {
        shoot_control.shoot_mode = SHOOT_READY;
    }

    /*测发弹延迟*/
//    cali_shoot_delay_time();
    //如果云台状态是 无力状态，就关闭射击
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    last_s = shoot_control.shoot_remote_data->rc_ctrl_t->rc.s[SHOOT_RC_MODE_CHANNEL];

}
//void shoot_heart_control_low(void) {

//    if((shoot_control.shoot_referee_data.heat + SHOOT_HEAT_REMAIN_VALUE) < 0.7f * shoot_control.shoot_referee_data.heat_limit)
//        shoot_control.trigger_speed_set = - SHOOT_LOW_TRIGGER_SPEED;
//    else if((shoot_control.shoot_referee_data.heat + SHOOT_HEAT_REMAIN_VALUE) < shoot_control.shoot_referee_data.heat_limit)
//        shoot_control.trigger_speed_set = - (3.0f - 2.857f*(shoot_control.shoot_referee_data.heat + SHOOT_HEAT_REMAIN_VALUE)
//                                             / shoot_control.shoot_referee_data.heat_limit) * SHOOT_LOW_TRIGGER_SPEED;
//    else
//        shoot_control.trigger_speed_set = - 0.15f * SHOOT_LOW_TRIGGER_SPEED;

//}
void  shoot_heart_control_high(void) {

//	if(shoot_control.shoot_referee_data.heat < 0.95 * (shoot_control.shoot_referee_data.heat_limit - shoot_control.shoot_referee_data.cooling_value))
//	{
//		shoot_control.trigger_speed_set = - SHOOT_HIGH_TRIGGER_SPEED;
//	}else if(shoot_control.shoot_referee_data.heat < 0.95 * shoot_control.shoot_referee_data.heat_limit){
//		shoot_control.trigger_speed_set = shoot_control.shoot_referee_data.cooling_value * 2 * PI / 10 / 8;
//
//	}else{
//	   shoot_control.trigger_speed_set = -0.1f * SHOOT_LOW_TRIGGER_SPEED;
//	}
		if(gimbal_control.gimbal_mode == GIMBAL_MODE_PC)
		{
				if(shoot_control.shoot_referee_data.heat_limit - shoot_control.shoot_referee_data.heat > SHOOT_HEAT_REMAIN_VALUE_1
					&& fire_heat_flag == 0)
				{
						shoot_control.trigger_speed_set = - SHOOT_HIGH_TRIGGER_SPEED; 				
				}
		    else
				{
						shoot_control.trigger_speed_set = 0.0f;		
						fire_heat_flag = 1;					
				}
				
				if(shoot_control.shoot_referee_data.heat_limit - shoot_control.shoot_referee_data.heat > SHOOT_HEAT_REMAIN_VALUE_2)
				{
						fire_heat_flag = 0;
				}
							
		}
		else
		{
				if((shoot_control.shoot_referee_data.heat + SHOOT_HEAT_REMAIN_VALUE) < 0.7f * shoot_control.shoot_referee_data.heat_limit)
						shoot_control.trigger_speed_set = - SHOOT_LOW_TRIGGER_SPEED;
		    else if((shoot_control.shoot_referee_data.heat + SHOOT_HEAT_REMAIN_VALUE) < 0.9f*shoot_control.shoot_referee_data.heat_limit)
		        shoot_control.trigger_speed_set = - 0.5f* SHOOT_LOW_TRIGGER_SPEED;
		    else
						shoot_control.trigger_speed_set = - 0.1f * SHOOT_LOW_TRIGGER_SPEED;		
		}
}
//数据更新，包括摩擦轮速度，拨弹电机转速，连发长按计时
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;
    static float32_t bullet_speed_fliter_1 = 0.0f;
    static float32_t bullet_speed_fliter_2 = 0.0f;
    static float32_t bullet_speed_fliter_3 = 0.0f;
    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //二阶低通滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.trigger_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.trigger_speed = speed_fliter_3;
    bullet_speed_fliter_1 = bullet_speed_fliter_2;
    bullet_speed_fliter_2 = bullet_speed_fliter_3;
    bullet_speed_fliter_3 = bullet_speed_fliter_2 * fliter_num[0] +
                            bullet_speed_fliter_1 * fliter_num[1] +
                            (fire_speed.fire_speed_value) * fliter_num[2];
    shoot_control.bullet_speed = bullet_speed_fliter_3;
    shoot_control.bullet_speed_delta = shoot_control.bullet_speed_set - shoot_control.bullet_speed;
    //	Slid_avg_filter(&fire_speed_avg_filter,shoot_control.bullet_speed_delta);
    shoot_control.fric_speed[0] = shoot_control.fric1_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED;
    shoot_control.fric_speed[1] = shoot_control.fric2_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED;

    //鼠标按键
    shoot_control.shoot_remote_data->mouse_press_l.last_press = shoot_control.shoot_remote_data->mouse_press_l.press;
    shoot_control.shoot_remote_data->mouse_press_r.last_press = shoot_control.shoot_remote_data->mouse_press_r.press;
    shoot_control.shoot_remote_data->mouse_press_l.press =  shoot_control.shoot_remote_data->rc_ctrl_t->mouse.press_l;
    shoot_control.shoot_remote_data->mouse_press_r.press =  shoot_control.shoot_remote_data->rc_ctrl_t->mouse.press_r;
    //长按计时
    if (shoot_control.shoot_remote_data->mouse_press_l.press)
    {
        if (shoot_control.shoot_remote_data->mouse_press_l.press_time < PRESS_LONG_TIME)
        {
            shoot_control.shoot_remote_data->mouse_press_l.press_time++;
        }
    }
    else
    {
        shoot_control.shoot_remote_data->mouse_press_l.press_time = 0;
    }
    if (shoot_control.shoot_remote_data->mouse_press_r.press)
    {
        if (shoot_control.shoot_remote_data->mouse_press_r.press_time < PRESS_LONG_TIME)
        {
            shoot_control.shoot_remote_data->mouse_press_r.press_time++;
        }
    }
    else
    {
        shoot_control.shoot_remote_data->mouse_press_r.press_time = 0;
    }
    //射击开关下档时间计时
    if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_remote_data->rc_ctrl_t->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {

        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time++;
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }
}

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{

    //每次拨动 1/4PI的角度
    if (shoot_control.move_flag == 0)
    {
        shoot_control.trigger_angle_set = rad_format(shoot_control.trigger_angle - PI_FOUR);
        shoot_control.move_flag = 1;
    }
    //到达角度判断
    if (rad_format(shoot_control.trigger_angle_set - shoot_control.trigger_angle) > 0.05f)
    {
        shoot_control.trigger_speed_set = shoot_PID_calc(&shoot_control.trigger_angle_pid,shoot_control.trigger_angle,shoot_control.trigger_angle_set);
    }
    else
    {
        shoot_control.move_flag = 0;
        shoot_control.shoot_mode = SHOOT_DONE;
    }
}

/**
  * @brief          堵转倒转检查
  * @param[in]      void
  * @retval         void
  */
static void trigger_block_check(void)
{
    if(shoot_control.shoot_mode == SHOOT_BULLET ||shoot_control.shoot_mode == SHOOT_ADJUST)
    {
        //角度没达到设定角度
        if(fabs(shoot_control.trigger_angle - shoot_control.trigger_angle_set ) > TRIGGER_BLOCK_ANGLE)
        {
            shoot_control.block_time++;
            if(shoot_control.block_time == TRIGGER_BLOCK_TIME) {
                block_flag = 1;
            }
        }
        else
        {
            block_flag = 0;
            shoot_control.block_time = 0;
        }
    }
    else if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
    {
        if(fabs(shoot_control.trigger_speed - shoot_control.trigger_speed_set ) > TRIGGER_BLOCK_SPEED)
        {
            shoot_control.block_time++;
            if(shoot_control.block_time == TRIGGER_BLOCK_TIME)
                block_flag = 1;
        }
        else
        {
            block_flag = 0;
            shoot_control.block_time = 0;
        }
    }
}
//返回摩擦轮电流值
int16_t get_fric1(void)
{
    return shoot_control.fric_given_current[0];
}
int16_t get_fric2(void)
{
    return shoot_control.fric_given_current[1];
}
void shoot_trigger_angle_update(void) {
    if (shoot_control.trigger_motor_measure->ecd - shoot_control.trigger_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        shoot_control.trigger_angle_per -= 2.0f * PI_ONE / REDUCTION_RATIO;//反转
    }
    else if (shoot_control.trigger_motor_measure->ecd - shoot_control.trigger_motor_measure->last_ecd < -HALF_ECD_RANGE)//此时ecd超过ECD_RANGE，从0开始增加
    {
        shoot_control.trigger_angle_per += 2.0f * PI_ONE / REDUCTION_RATIO;//正转
    }
    if (shoot_control.trigger_angle_per >= PI_ONE)//保证trigger_angle在-PI和PI之间
    {
        shoot_control.trigger_angle_per -= 2.0f * PI_ONE;
    }
    else if (shoot_control.trigger_angle_per <= -PI_ONE)
    {
        shoot_control.trigger_angle_per += 2.0f * PI_ONE;
    }
    //计算输出轴角度
    shoot_control.trigger_angle = shoot_control.trigger_angle_per + shoot_control.trigger_motor_measure->ecd * MOTOR_ECD_TO_ANGLE / REDUCTION_RATIO;
}
//连发后调整模式角度设定值计算（为了达到单发的合适角度）
static void shoot_adjust_control(void)
{
    if(block_flag == 1)
    {
        shoot_control.trigger_angle_set = shoot_control.trigger_angle - PI_FOUR/16;//反转只控制了速度，set_angle没变，故使用当前角度控制，与单发拨弹逻辑区别
        shoot_control.shoot_mode = SHOOT_READY;
        block_flag = 0;
    }
}

//uint8_t shoot_num = 0;
//uint16_t shoot_delay_count;
//uint32_t shoot_single_start_cnt;
//uint32_t shoot_continue_start_cnt;
//uint16_t shoot_delay_time_single = 0;
//uint16_t shoot_delay_time_continue = 0;
//static void cali_shoot_delay_time(void)
//{
//	/*测试单发模式*/
//	/*软件开始发弹*/
//    if((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)))
//		{
////				shoot_start_flag = 1;
//			  shoot_single_start_cnt = DWT->CYCCNT;
//		}
//	/*发弹后计时*/
//	/*裁判系统检测到后停止计时*/
//	//读取发弹与裁判系统监测之间的dwt定时器计时数从而计算发弹延迟
//    if (bullet_speed_corret_flag)
//		{
//			shoot_delay_time_single = (DWT->CYCCNT - shoot_single_start_cnt)/168000;//主频为168MHz，换算为毫秒
//			if(continue_shoot_end_flag == 0)
//			{
//				shoot_delay_time_continue = (DWT->CYCCNT - shoot_continue_start_cnt)/168000;
//				continue_shoot_end_flag = 1;
//			}
//			bullet_speed_corret_flag = 0;
////			shoot_start_flag = 0;
//
//		}
//
//	 /*测试连发模式*/
//		if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
//		{
//			if( continue_shoot_start_flag == 0 )
//			{
//			  shoot_continue_start_cnt = DWT->CYCCNT;
//				continue_shoot_start_flag = 1;
//				continue_shoot_end_flag = 0;
//			}
//				}
////				/*裁判系统检测到后停止计时*/
//
//		if(!switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))	continue_shoot_start_flag = 0;
//}
