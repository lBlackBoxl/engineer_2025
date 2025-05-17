#ifndef INIT_H
#define INIT_H

#include "main.h"

//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
#define CHASSIS_CONTROL_TIME_MS 2
//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0
//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 2
//选择底盘状态 开关通道号
#define RC_S_RIGHT 0
#define RC_S_LEFT 1
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN -0.005f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN -0.004f
//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN    -0.000009f
#define CHASSIS_WZ_MOUSE_SEN -0.00008f
//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000003f
//底盘速度设定值滤波器
#define CHASSIS_ACCEL_X_NUM 0.18f
#define CHASSIS_ACCEL_Y_NUM 0.18f
//摇杆死区
#define CHASSIS_RC_DEADLINE 10
#define MOUSE_DEADBAND 	0
//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 					 		3.5f
#define SHIFT_NORMAL_MAX_CHASSIS_SPEED_X 		2.5f
#define SHIFT_NORMAL_MAX_CHASSIS_SPEED_Y 		1.8f
#define NORMAL_MAX_CHASSIS_SPEED_X 		2.5f
#define NORMAL_MAX_CHASSIS_SPEED_Y 		1.8f
#define SHIFT_SLOW_CHASSIS_SPEED_X 		1.0f
#define SHIFT_SLOW_CHASSIS_SPEED_Y 		1.0f
#define SLOW_CHASSIS_SPEED_X 					0.8f
#define SLOW_CHASSIS_SPEED_Y 			 		0.3f
#define SELF_CONTROL_SPEED_X 					0.7f
#define SELF_CONTROL_SPEED_Y 			 		0.7f
//麦轮解算参数
#define CHASSIS_WZ_SET_SCALE 		   0.0f
#define MOTOR_DISTANCE_TO_CENTER   0.307f
//麦轮反解算参数
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f
//底盘电机速度环PID
#define CHASSIS_M3508_SPEED_PID_KP 				32000.0f
#define CHASSIS_M3508_SPEED_PID_KI        1000.0f
#define CHASSIS_M3508_SPEED_PID_KD        1.0f
#define CHASSIS_M3508_SPEED_PID_MAX_OUT   8000.0f
#define CHASSIS_M3508_SPEED_PID_MAX_IOUT  3000.0f
//底盘角速度环PID
#define CHASSIS_YAW_PID_KP       16.5f
#define CHASSIS_YAW_PID_KI       0.5f
#define CHASSIS_YAW_PID_KD 			 0.28f
#define CHASSIS_YAW_PID_MAX_OUT  5.8f
#define CHASSIA_YAW_PID_MAX_IOUT 0.3f
//夹矿电机
#define CLAMP_POSITION_PID_KP      20.0f
#define CLAMP_POSITION_PID_KI      0.0f
#define CLAMP_POSITION_PID_KD      0.05f
#define CLAMP_POSITION_PID_MAX_OUT 20.0f
#define CLAMP_POSITION_PID_MAX_KD  0.0f
#define CLAMP_SPEED_PID_KP       	 1500.0f
#define CLAMP_SPEED_PID_KI         0.0f
#define CLAMP_SPEED_PID_KD         0.0f
#define CLAMP_SPEED_PID_MAX_OUT    10000.0f
#define CLAMP_SPEED_PID_MAX_KD     0.0f
//编码器参考值
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8192
//减速比
#define REDUCTION_RATIO_3508				187.0f/3591.0f
#define REDUCTION_RATIO_2006				1.0f/36.0f
//电机单位转化参数
#define MOTOR_ECD_TO_ANGLE_2006     0.000021305288720633f
#define MOTOR_ECD_TO_ANGLE_3508     0.00003994074176199f
#define RPM_TO_RAD_S							  0.104719755119659774f
//4310角度参数
#define MOTOR_ANGLE_SET_4310_INIT   0.0f
#define MOTOR_ANGLE_SET_4310_TEST   0.6f
//4310速度参数
#define MOTOR_SPEED_SET_4310    5.0f
//机械臂相关参数
#define RAD_TO_DEG  57.29577951308232088f

#define LINK1_LENGTH 260.0f
#define LINK2_LENGTH 235.0f

#define motor_to_real 23.873241463f
#define sc_to_arm  2.6f
#define arm_length 600.89f
#endif // INIT_H
