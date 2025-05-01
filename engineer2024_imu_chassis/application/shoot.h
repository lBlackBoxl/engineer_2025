#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"
#include "remote_control.h"
#include "can_communicate.h"
#include "pid.h"
#include "adrc.h"
#include "controller.h"
#include "pc_task.h"
#include "td.h"
//****************************固定值***********************************//
//与拨弹相关的角度
#define PI_ONE											3.141592653589793f
#define PI_FOUR                     0.78539816339744830961566084581988f
#define SHOOT_ADJUST_ANGLE          PI_FOUR/3
//校准后的减速比
#define REDUCTION_RATIO  						36.0f
//编码器值转换为弧度的系数
#define MOTOR_ECD_TO_ANGLE          0.0007669903939f 

//电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596f

//射击摩擦轮激光打开 关闭
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E

//****************************调整值***********************************//
//鼠标长按判断
#define PRESS_LONG_TIME             0.0f

//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME              2000
#define PC_TIME                     50
//摩擦轮高速 加速 时间
#define UP_ADD_TIME                 80
//摩擦轮速度
#define BULLET_SPEED_SET          	28.0f
#define SHOOT_FIRE_SPEED_HIGH       21.0f//单发
#define SHOOT_FIRE_SPEED_LOW        20.5f //连发
#define BULLET_SPEED_FIX_K          0.1f           
//射击发射开关通道数据
#define SHOOT_RC_MODE_CHANNEL       1

//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME     15

//拨弹速度
#define SHOOT_ADJUST_SPEED            2.0f 
#define SHOOT_HIGH_TRIGGER_SPEED      15.0f 
#define SHOOT_LOW_TRIGGER_SPEED       11.0f 
//堵转角度
#define TRIGGER_BLOCK_ANGLE					0.1f
//堵转速度
#define TRIGGER_BLOCK_SPEED			    1.0f	
#define TRIGGER_BLOCK_TIME          500

//拨弹轮电机速度环，角度环PID
#define TRIGGER_SPEED_PID_KP        1200.0f
#define TRIGGER_SPEED_PID_KI        800.0f
#define TRIGGER_SPEED_PID_KD        1.0f
#define TRIGGER_SPEED_PID_MAX_OUT   10000.0f
#define TRIGGER_SPEED_PID_MAX_IOUT  2000.0f
#define TRIGGER_ANGLE_PID_KP          20.0f
#define TRIGGER_ANGLE_PID_KI          2.0f
#define TRIGGER_ANGLE_PID_KD          0.1f
#define TRIGGER_ANGLE_PID_MAX_OUT     8.0f
#define TRIGGER_ANGLE_PID_MAX_IOUT    0.5f
//摩擦轮电机速度环PID
#define FRIC_SPEED_PID_KP        	7000.0f
#define FRIC_SPEED_PID_KI        	1.0f
#define FRIC_SPEED_PID_KD        	0.01f
#define FRIC_SPEED_PID_MAX_OUT      16383.0f
#define FRIC_SPEED_PID_MAX_IOUT   	2000.0f

#define FRIC_DEADBAND    0.2

//热量限制余量

#define SHOOT_HEAT_REMAIN_VALUE     10
#define SHOOT_HEAT_REMAIN_VALUE_1     40
#define SHOOT_HEAT_REMAIN_VALUE_2     80

typedef enum
{
    SHOOT_STOP = 0,     //发射停止
    SHOOT_READY_FRIC,  //等待摩擦轮
    SHOOT_READY,      //发射等待
    SHOOT_BULLET,   //单发
    SHOOT_CONTINUE_BULLET,//连发
    SHOOT_DONE,   //射击完成
	  SHOOT_ADJUST,  //射击调整
} shoot_mode_e;
typedef struct{
	const rx_data_t *pc_rx_data;   //开火
	uint8_t pc_continue_flag ;
	uint16_t pc_continue_count;
}shoot_pc_t;
typedef struct{
		uint16_t heat_limit;
    uint16_t heat;
	  uint16_t cooling_value;
}shoot_referee_t;
typedef struct
{
    shoot_mode_e shoot_mode;
	  shoot_mode_e shoot_mode_last;
    sREMOTE_data *shoot_remote_data;
	  const RC_ctrl_t *shoot_rc;
    const motor_measure_t *trigger_motor_measure;
    const motor_measure_t *fric1_motor_measure;
    const motor_measure_t *fric2_motor_measure;
    PID_t trigger_speed_pid;
    pid_type_def trigger_angle_pid;
    PID_t fric_motor_pid[2];
    float32_t bullet_speed;                          //弹速
	  float32_t bullet_speed_delta;                    //弹速与实际弹速的差
    float32_t bullet_speed_fix;                      //弹速修正系数
  	float32_t bullet_speed_set;                      //设定弹速
    fp32 trigger_speed_set;   //拨弹轮的拨动速度
	  fp32 trigger_speed;	
		fp32 trigger_angle;
	  fp32 trigger_angle_per;
    fp32 trigger_angle_set;
	  TD_t      fric_speed_set_td;
    fp32 fric_speed[2];
    fp32 fric_speedset[2];		    
    fp32 fric_speed_set;
    int16_t fric_given_current[2];
		 
    int16_t trigger_given_current;

    uint16_t rc_s_time;//拨码开关时间

    uint16_t block_time;
    uint16_t reverse_time;
    bool_t move_flag;
		
		shoot_referee_t  shoot_referee_data;
    shoot_pc_t  shoot_pc_data;
} shoot_control_t;

extern uint8_t shoot_fire_state;	
extern uint8_t block_flag;
extern shoot_control_t shoot_control;//射击数据
extern void shoot_init(void);
extern int16_t shoot_control_loop(void);
extern int16_t get_fric1(void);
extern int16_t get_fric2(void);
extern void shoot_trigger_angle_update(void);
#endif

