#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "struct_typedef.h"
#include "remote_control.h"
#include "can_communicate.h"
#include "adrc.h"
#include "pid.h"
#include "kalman_filter.h"
#include "TD.h"
#include "controller.h"
#include "gimbal_behaviour.h"
#define GIMBAL_YAW_OFFSET_ECD  836
//看绝对值角度
#define GIMBAL_PITCH_ABSOLUTE_ANGLE_MAX 0.6f
#define GIMBAL_PITCH_ABSOLUTE_ANGLE_MIN -0.36f

//yaw速度环PID参数以及PID最大输出，积分输出
#define YAW_SPEED_PID_KP       	35000.0f 
#define YAW_SPEED_PID_KI        3000.0f 
#define YAW_SPEED_PID_KD        0.5f
#define YAW_SPEED_PID_MAX_OUT   30000.0f
#define YAW_SPEED_PID_MAX_IOUT  1000.0f
//yaw角度环角度由陀螺仪解算PID参数以及PID最大输出，积分输出
#define YAW_ABSOLUTE_ANGLE_PID_KP        16.0f   
#define YAW_ABSOLUTE_ANGLE_PID_KI        0.002f  
#define YAW_ABSOLUTE_ANGLE_PID_KD        0.12f  
#define YAW_ABSOLUTE_ANGLE_PID_MAX_OUT   9.0f
#define YAW_ABSOLUTE_ANGLE_PID_MAX_IOUT  0.3f 
//pitch角度环角度由陀螺仪解算PID参数以及PID最大输出，积分输出
#define PITCH_ABSOLUTE_ANGLE_PID_KP 			  22.0f
#define PITCH_ABSOLUTE_ANGLE_PID_KI 		    0.0015f
#define PITCH_ABSOLUTE_ANGLE_PID_KD 			  0.15f
#define PITCH_ABSOLUTE_ANGLE_PID_MAX_OUT 	  6.0f
#define PITCH_ABSOLUTE_ANGLE_PID_MAX_IOUT 	0.1f
//pitch速度环PID参数以及PID最大输出，积分输出
#define PITCH_SPEED_PID_KP        	4.2f
#define PITCH_SPEED_PID_KI        	0.1f
#define PITCH_SPEED_PID_KD        	0.001f
#define PITCH_SPEED_PID_MAX_OUT		  10.0f
#define PITCH_SPEED_PID_MAX_IOUT 	  0.3f


//云台是否使用ADRC控制
#define ADRC_MODE 0
// PC自动瞄准灵敏度（正负号代表方向）
#define YAW_PC_SEN 1
#define PITCH_PC_SEN 1
//yaw,pitch控制通道以及状态开关通道（遥控器）
#define YAW_CHANNEL			2
#define PITCH_CHANNEL 		3
#define GIMBAL_MODE_CHANNEL	0
//按X掉头180°
#define TURN_KEYBOARD 	KEY_PRESSED_OFFSET_X
//掉头云台速度
#define TURN_SPEED	0.005f
//遥控器与鼠标输入死区
#define RC_DEADBAND		10
#define MOUSE_DEADBAND 	0
//遥控器与鼠标灵敏度（正负号代表方向）
#define YAW_RC_SEN 		-0.000013f
#define PITCH_RC_SEN 	0.000008f
#define YAW_MOUSE_SMALLSEN   -0.00015f
#define PITCH_MOUSE_SMALLSEN -0.00015f

//任务初始化延时
#define GIMBAL_TASK_INIT_TIME	1000
//云台任务的周期（1ms）
#define GIMBAL_CONTROL_TIME	1
//yaw,pitch轴电机输出电流的正负
#define PITCH_TURN 	1
#define YAW_TURN 	0
//电机码盘值最大以及中值
#define HALF_ECD_RANGE 	4096
#define ECD_RANGE 		8191
//电机编码值转化成角度值，转换公式为2*PI/8192
#define MOTOR_ECD_TO_RAD 0.000766990394f
//yaw轴电机超时
#define TIMEOUT_MAX 1000
//pitch轴力矩
#define PITCH_M                                         1.8f//kg
#define PITCH_L                                         0.05f//m
#define GRAVITY                                         9.8f
#define KT                                              0.87f//N*m/A小米电机力矩电流转换系数
//角度环PID结构体
typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;
    fp32 set;
    fp32 get;
    fp32 err;
    fp32 max_out;
    fp32 max_iout;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 out;
} gimbal_PID_t;
//电机结构体
typedef struct
{
    const motor_measure_t *gimbal_motor_measure;//电机数据（电流，编码值）
    gimbal_PID_t gimbal_motor_absolute_angle_pid; //绝对值PID
    PID_t gimbal_motor_gyro_pid;  //陀螺仪PID
    TD_t PC_Control_Td;
    uint16_t offset_ecd;
	  fp32 absolute_angle_offset;
    //以下角度、角速度均为弧度制
		fp32 max_absolute_angle;
    fp32 min_absolute_angle;
    fp32 absolute_angle;
    fp32 absolute_angle_set;
    fp32 motor_gyro;
    fp32 motor_gyro_set;
		float32_t gravity_feedforward_current;
	
    fp32 raw_cmd_current;
    fp32 current_get;
    fp32 current_last;
    fp32 current_set;
    fp32 given_current;
} gimbal_motor_t;
//云台结构体
typedef struct
{
	  gimbal_mode_e gimbal_mode_last;
    gimbal_mode_e gimbal_mode;
	  sREMOTE_data *gimbal_remote_data;//遥控器
    const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
    gimbal_motor_t gimbal_yaw_motor;  //Y轴模式
    gimbal_motor_t gimbal_pitch_motor; //P轴模式
	
		float dt;
		uint32_t dwt_count;
} gimbal_control_t;

extern gimbal_control_t gimbal_control;

#endif
