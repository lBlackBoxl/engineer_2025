#include "motor_task.h"
#include "bsp_dwt.h"
#include "freertos.h"
#include "cmsis_os.h"

#define motor_to_real 23.873241463f
#define sc_to_arm 2.9f
#define arm_length 665.0f
static void armend_control_init(self_control_t *armend_control_init);
static void motor_feedback_update(self_control_t *armend_feedback);
static void motor_set_position(self_control_t *armend_set_position);

self_control_t armend;
float dt1 = 0.0f;
uint32_t  DWT_Count1 = 0;

fp32 self_control_motor_position[3];

void motor_task(void const * argument)
{
	armend_control_init(&armend);
	uint32_t system_clock = osKernelSysTick();
	osDelay(1);
	
	while(1)
	{
		dt1 = DWT_GetDeltaT(&DWT_Count1);		
		motor_feedback_update(&armend);
		
					if(armend.motor_DM_data[2].DM_motor_measure->motor_enabled != 1)
					{
						CAN_cmd_4310_3_init();					
					}
					else if(armend.motor_DM_data[2].DM_motor_measure->motor_enabled == 1)
					{
						if(armend.motor_DM_data[2].DM_motor_measure->motor_speed > 0.1f)
						{
							CAN_DM_TRIGGER_3_send_MIT(0.0f, 0.0f,0.0f,0.0f,0.11f);//sin_calc(0.5,0.0002)			
						}
						else if(armend.motor_DM_data[2].DM_motor_measure->motor_speed < -0.1f)
						{
							CAN_DM_TRIGGER_3_send_MIT(0.0f, 0.0f,0.0f,0.0f,-0.11f);//sin_calc(0.5,0.0002)
						}
						else
						{
							CAN_DM_TRIGGER_3_send_MIT(0.0f, 0.0f,0.0f,0.0f,0.0f);//sin_calc(0.5,0.0002)
						}
					}
					DWT_Delay(0.0003f);
					
					if(armend.motor_DM_data[1].DM_motor_measure->motor_enabled != 1)
					{
						CAN_cmd_4310_2_init();					
					}
					else if(armend.motor_DM_data[1].DM_motor_measure->motor_enabled == 1)
					{
						//CAN_cmd_4310_2(armend.motor_DM_data[1].position_set, armend.motor_DM_data[1].speed_set);
						
						if(armend.motor_DM_data[1].DM_motor_measure->motor_speed > 0.1f)
						{
							CAN_DM_TRIGGER_2_send_MIT(0.0f, 0.0f,0.0f,0.0f,-0.03f);//sin_calc(0.5,0.0002)			
						}							
						else if(armend.motor_DM_data[1].DM_motor_measure->motor_speed < -0.05f)
						{
							CAN_DM_TRIGGER_2_send_MIT(0.0f, 0.0f,0.0f,0.0f,-0.15f);//sin_calc(0.5,0.0002)
						}
						else
						{
							CAN_DM_TRIGGER_2_send_MIT(0.0f, 0.0f,0.0f,0.0f,-0.09f);//sin_calc(0.5,0.0002)
						}						
					}
					DWT_Delay(0.0003f);
					
					if(armend.motor_DM_data[0].DM_motor_measure->motor_enabled != 1)
					{
						CAN_cmd_4310_1_init();					
					}
					else if(armend.motor_DM_data[0].DM_motor_measure->motor_enabled == 1)
					{
							//CAN_cmd_4310_1(armend.motor_DM_data[0].position_set, armend.motor_DM_data[0].speed_set);	
							if(armend.motor_DM_data[0].DM_motor_measure->motor_speed > 0.05f)
						{
							CAN_DM_TRIGGER_1_send_MIT(0.0f, 0.0f,0.0f,0.0f,0.00015f);//sin_calc(0.5,0.0002)			
						}							
						else if(armend.motor_DM_data[0].DM_motor_measure->motor_speed < -0.05f)
						{
							CAN_DM_TRIGGER_1_send_MIT(0.0f, 0.0f,0.0f,0.0f,-0.00015f);//sin_calc(0.5,0.0002)
						}
						else
						{
							CAN_DM_TRIGGER_1_send_MIT(0.0f, 0.0f,0.0f,0.0f,0.0f);//sin_calc(0.5,0.0002)
						}							
					}
//					else
//					{
//						CAN_cmd_4310_1_disable();
//					}
					DWT_Delay(0.0005f);

					
		osDelay(1);			
	}
}

//arm 4-5-6
void armend_control_init(self_control_t *armend_control_init)
{
	osDelay(200);
	CAN_cmd_4310_1_init();
	osDelay(1);
	CAN_cmd_4310_2_init();
	osDelay(1);
	CAN_cmd_4310_3_init();
	osDelay(1);
	CAN_cmd_4310_1_setzero();
	osDelay(1);
	CAN_cmd_4310_2_setzero();
	osDelay(1);
	CAN_cmd_4310_3_setzero();
	for(int i = 0; i < 3; i++)
	{
		armend_control_init->motor_DM_data[i].DM_motor_measure = return_4310_measure(i);
	}
	osDelay(50);
	armend_control_init->motor_DM_data[0].position_set = armend_control_init->motor_DM_data[0].DM_motor_measure->motor_position;
	armend_control_init->motor_DM_data[1].position_set = armend_control_init->motor_DM_data[1].DM_motor_measure->motor_position;
	armend_control_init->motor_DM_data[2].position_set = armend_control_init->motor_DM_data[2].DM_motor_measure->motor_position;
	armend_control_init->motor_DM_data[0].speed_set = 0;
	armend_control_init->motor_DM_data[1].speed_set = 0;
	armend_control_init->motor_DM_data[2].speed_set = 0;

	motor_feedback_update(armend_control_init);

	osDelay(100);
	
	DWT_Delay(0.0005f);
//	CAN_cmd_4310_1_disable();
//	osDelay(1);
//	CAN_cmd_4310_2_disable();	
//	osDelay(1);
//	CAN_cmd_4310_3_disable();	
}
float32_t k;
void motor_feedback_update(self_control_t *armend_feedback)
{

//	k = (603.3f/14.9f);
//	self_control_motor_position[0] = arm_length - armend_feedback->motor_DM_data[0].DM_motor_measure->motor_position * motor_to_real * sc_to_arm ;//先换算到自定义控制器真实长度，再换算到机械臂真实长度，再减去偏置
//	self_control_motor_position[1] = -armend_feedback->motor_DM_data[2].DM_motor_measure->motor_position * motor_to_real * sc_to_arm ;
//	self_control_motor_position[2] = -(armend_feedback->motor_DM_data[1].DM_motor_measure->motor_position * motor_to_real * sc_to_arm)/k ;//抬升范围为0- -14.9f
		self_control_motor_position[0] = armend_feedback->motor_DM_data[0].DM_motor_measure->motor_position;
		self_control_motor_position[1] = armend_feedback->motor_DM_data[2].DM_motor_measure->motor_position;
		self_control_motor_position[2] = armend_feedback->motor_DM_data[1].DM_motor_measure->motor_position;
}

void motor_set_position(self_control_t *armend_set_position)
{
		armend_set_position->motor_DM_data[0].position_set = (fp32)0.0f;//(armend_set_position->arm_4_TD.x);
		armend_set_position->motor_DM_data[1].position_set = (fp32)0.0f;//(armend_set_position->arm_5_TD.x);
		armend_set_position->motor_DM_data[2].position_set = (fp32)0.0f;//(armend_set_position->arm_5_TD.x);
		
		if(armend_set_position->motor_DM_data[0].position_set - armend_set_position->motor_DM_data[0].DM_motor_measure->motor_position < 0.0f)
		{
			armend_set_position->motor_DM_data[0].speed_set = -4.0f;
		}
		else 
		{
			armend_set_position->motor_DM_data[0].speed_set = 4.0f;
		}

		if(armend_set_position->motor_DM_data[1].position_set - armend_set_position->motor_DM_data[1].DM_motor_measure->motor_position < 0.0f)
		{
			armend_set_position->motor_DM_data[1].speed_set = -4.0f;
		}
		else 
		{
			armend_set_position->motor_DM_data[1].speed_set = 4.0f;
		}
		
		if(armend_set_position->motor_DM_data[2].position_set - armend_set_position->motor_DM_data[2].DM_motor_measure->motor_position < 0.0f)
		{
			armend_set_position->motor_DM_data[2].speed_set = -4.0f;
		}
		else 
		{
			armend_set_position->motor_DM_data[2].speed_set = 4.0f;
		}
}

