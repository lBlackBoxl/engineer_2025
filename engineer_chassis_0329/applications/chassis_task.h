#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "struct_typedef.h"
#include "user_lib.h"
#include "can_communicate.h"
#include "controller.h"
#include "TD.h"
#include "main.h"
#include "keyboard.h"
#include "referee.h"
#include "nx_communicate.h"
#include "referee_usart_task.h"

extern void chassis_task(void const *pvParameters);
extern uint8_t arm_restart_flag;

typedef enum
{
	NO_POWER_MODE,
	RUN_MODE,
} chassis_mode_e;

typedef enum
{
	NX_CONTROL_MODE,
	ONE_KEY_MODE,
	SELF_CONTROL_MODE,
} arm_mode_e;

typedef enum
{
	Home,
	Au,
	Ag,
	Exchange,
	GouDong,
  Wait
} move_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} chassis_motor_t;

typedef struct
{
	const motor_DM_motor_t *DM_motor_measure;
	fp32 position_set;
	fp32 speed_set;
}motor_DM_t;

typedef struct
{
	const motor_measure_t *motor_measure;
	fp32 		speed;
	fp32 		speed_set;	
	fp32 		position;
	fp32 		position_set;
	fp32 		offset_ecd;
	int32_t round_cnt;
	int16_t give_current;
}position_motor_t;

typedef struct
{
	const RC_ctrl_t *chassis_RC;                //����ʹ�õ�ң����ָ��
  chassis_motor_t motor_chassis[4];           //����3508�������
	position_motor_t motor_clamp;								//�п�2006�������
	position_motor_t motor_uwb;								//uwb2006�������
	motor_DM_t motor_DM_data;          			//����������

	
	PID_t chassis_motor_speed_pid[4];           //����3508PID����
	PID_t chassis_yaw_pid;                      //������ת����PID����
	PID_t clamp_motor_position_pid;
	PID_t clamp_motor_speed_pid;
	PID_t uwb_motor_position_pid;
	PID_t uwb_motor_speed_pid;
	
	TD_t chassis_yaw_TD;                        //������ת΢���㷨����
	TD_t joint_TD;
	
	first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-point.ʹ��һ�׵�ͨ�˲������趨ֵ
  first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point.ʹ��һ�׵�ͨ�˲������趨ֵ
	
	fp32 vx;                          //chassis_t vertical speed, positive means forward,unit m/s. �����ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy;                          //chassis_t horizontal speed, positive means letf,unit m/s.�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  fp32 wz;                          //chassis_t rotation speed, positive means counterclockwise,unit rad/s.������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 vx_set;                      //chassis_t set vertical speed,positive means forward,unit m/s.�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy_set;                      //chassis_t set horizontal speed,positive means left,unit m/s.�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  fp32 wz_set;                      //chassis_t set rotation speed,positive means counterclockwise,unit rad/s.�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s

	fp32 yaw_angle;
	fp32 yaw_angle_set;
	
	chassis_mode_e  chassis_mode;//����ģʽ
	chassis_mode_e  last_chassis_mode;	
	arm_mode_e arm_mode;//��е��ģʽ
	arm_mode_e last_arm_mode;
	move_mode_e move_mode;
	move_mode_e last_move_mode;
	uint16_t suker_key_flag;
	
	float dt;
	uint32_t  DWT_Count;	
	
	uint16_t error_info;
} chassis_t;

extern chassis_t chassis;

#endif // !CHASSIS_TASK_H
