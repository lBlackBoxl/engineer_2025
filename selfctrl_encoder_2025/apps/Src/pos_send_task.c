#include "pos_send_task.h"
#include "cmsis_os.h"
#include "protocol.h"
#include <string.h>
#include "encoder_task.h"
#include "referee.h"
#include "main.h"

J_Data_t* ArmData;
J_Data_t  ArmSendData;
HAL_StatusTypeDef referee_tx_status = HAL_BUSY;

static void joint_init(J_Data_t* arm);
static void motor_data_calc(J_Data_t* arm, J_Data_t arm_data);

void Referee_Task(void const * argument){
	osDelay(500);
	uart6_init();
	
	joint_init(&ArmSendData);
	ArmData = get_arm_ptr();
	joint_init(ArmData);
	
	osDelay(1000);

	for(;;){
		if(ArmData->status == UPDATED){
			motor_data_calc(&ArmSendData, *ArmData);
			referee_tx_status = (HAL_StatusTypeDef)referee_tx_send(&ArmSendData.joint[0]);
			if(referee_tx_status == HAL_OK){
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);		
				HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
			}
			else{
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);	
				HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
			}
			ArmData->status = UNUPDATE;
		}
		osDelay(3);
	}
}

void joint_init(J_Data_t* arm){
	//限位初始化
	arm->limit[0].min = JOINT_0_MINPOS;
	arm->limit[0].max = JOINT_0_MAXPOS;
	arm->limit[1].min = JOINT_1_MINPOS;
	arm->limit[1].max = JOINT_1_MAXPOS;
	arm->limit[2].min = JOINT_2_MINPOS;
	arm->limit[2].max = JOINT_2_MAXPOS;
	arm->limit[3].min = JOINT_3_MINPOS;
	arm->limit[3].max = JOINT_3_MAXPOS;
	arm->limit[4].min = JOINT_4_MINPOS;
	arm->limit[4].max = JOINT_4_MAXPOS;
	arm->limit[5].min = JOINT_5_MINPOS;
	arm->limit[5].max = JOINT_5_MAXPOS;
	
	//映射初始化
	arm->limit[0].ratio = JOINT_0_RATIO;
	arm->limit[0].offset= JOINT_0_OFFSET;
	arm->limit[1].ratio = JOINT_1_RATIO;
	arm->limit[1].offset= JOINT_1_OFFSET;
	arm->limit[2].ratio = JOINT_2_RATIO;
	arm->limit[2].offset= JOINT_2_OFFSET;
	arm->limit[3].ratio = JOINT_3_RATIO;
	arm->limit[3].offset= JOINT_3_OFFSET;
	arm->limit[4].ratio = JOINT_4_RATIO;
	arm->limit[4].offset= JOINT_4_OFFSET;
	arm->limit[5].ratio = JOINT_5_RATIO;
	arm->limit[5].offset= JOINT_5_OFFSET;
}

void motor_data_calc(J_Data_t* arm, J_Data_t arm_data){//换算成p1,p2单位毫米，r1-r5单位rad
	static uint8_t i;
	for(i = 0;i < 6;i++){
		//角度换算成rad，圈数也算成rad，最后全部累加到joint[i]里
		if(arm_data.cycle[i]<ENCODER_HALF_CYCLE_RESOLUTION){
			arm_data.joint[i] = (arm_data.cycle[i] + arm_data.joint[i]/360)*2*PI;

		}
		else if(arm_data.cycle[i] >= ENCODER_HALF_CYCLE_RESOLUTION){
			arm_data.joint[i] = (arm_data.cycle[i] - ENCODER_CYCLE_RESOLUTION + arm_data.joint[i]/360)*2*PI;
		}
				
		//线性映射和限位
		arm_data.joint[i] = arm_data.joint[i]*arm_data.limit[i].ratio + arm_data.limit[i].offset; 
		arm->joint[i] = DEADBAND_LIMIT(arm_data.joint[i], arm_data.limit[i].min, arm_data.limit[i].max);
	}
}
