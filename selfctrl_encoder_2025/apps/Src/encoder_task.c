#include "encoder_task.h"
#include "bsp_encoder.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "usart.h"

J_Data_t Arm;

static Encoder_Status_e joint_angle_update(void);
static Encoder_Status_e joint_cycle_update(void);
static void all_joint_reset(void);

void Encoder_Task(void const * argument){	
	encoder_buf_init();
	osDelay(1000);

	Arm.status = UNUPDATE;
	all_joint_reset();
	

	
	for(;;){	
		if(Arm.status == UNUPDATE){//等待发送完成
			Arm.status = UPDATING;
			//采样圈数（所有关节J1-6）
			if(joint_cycle_update() == ENCODER_ERROR){
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);		
				HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
				Arm.status = UNUPDATE;
				continue;
			}
			
			//采样角度（所有关节J1-7）
			if(joint_angle_update() == ENCODER_ERROR){
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);		
				HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
				Arm.status = UNUPDATE;
				continue;
			}
			Arm.status = UPDATED;
			osDelay(1);
		}
	}
}

void joint_reset(uint8_t joint_id){
	encoder_reset_init(&AllEncoder.Sender, joint_id);
	HAL_UART_Transmit_DMA(&huart1,(uint8_t*)&AllEncoder.Sender, sizeof(AllEncoder.Sender));
}

void all_joint_reset(void){
	uint8_t i;
	for(i=ADDRESS_OFFSET;i<(6+ADDRESS_OFFSET);i++){
		joint_reset(i);
		osDelay(5);
	}
}

void get_joint_angle(uint8_t joint_id){
	read_angle_init(&AllEncoder.Sender, joint_id);
	HAL_UART_Transmit_DMA(&huart1,(uint8_t*)AllEncoder.Sender.encoder.hex, sizeof(AllEncoder.Sender));
	//waiting for transmission
	osDelay(2);
}

Encoder_Status_e joint_angle_update(void){
	uint8_t i;
	Encoder_Status_e	err_code;
	err_code = ENCODER_OK;
	for(i=ADDRESS_OFFSET;i<(6+ADDRESS_OFFSET);i++){
		get_joint_angle(i);
		if(AllEncoder.Encoder[6-ADDRESS_OFFSET].rx_flag == 1){
			Arm.joint[i-ADDRESS_OFFSET] = AllEncoder.Encoder[i-ADDRESS_OFFSET].data.angle.fpData;
			AllEncoder.Encoder[i].rx_flag = 0;
		}
		else{
			AllEncoder.Encoder[i].rx_flag = 0;
			err_code = ENCODER_ERROR;
		}
	}
	return err_code;
}

void get_joint_cycle(uint8_t joint_id){
	
	read_cycle_init(&AllEncoder.Sender, joint_id);
	HAL_UART_Transmit_DMA(&huart1,(uint8_t*)AllEncoder.Sender.encoder.hex, sizeof(AllEncoder.Sender));
	//waiting for transmission
	osDelay(2);
}

Encoder_Status_e joint_cycle_update(void){
	uint8_t i;
	Encoder_Status_e	err_code;
	err_code = ENCODER_OK;
	for(i=ADDRESS_OFFSET;i<(6 + ADDRESS_OFFSET);i++){
		get_joint_cycle(i);
		if(AllEncoder.Encoder[i-ADDRESS_OFFSET].rx_flag == 1){
			Arm.cycle[i-ADDRESS_OFFSET] = AllEncoder.Encoder[i-ADDRESS_OFFSET].data.cycle;
			AllEncoder.Encoder[i].rx_flag = 0;
		}
		else{
			AllEncoder.Encoder[i].rx_flag = 0;
			err_code = ENCODER_ERROR;
		}
	}
	return err_code;
}

Encoder_Status_e encoder_test(uint8_t id){
	uint16_t i;
	Tx_t str;
	read_angle_init(&str, id);
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&str, sizeof(str));
	while(!AllEncoder.Encoder[id-ADDRESS_OFFSET].rx_flag){
		i++;
		if(i>20){
			return ENCODER_ERROR;
		}
		osDelay(1);
	}
	AllEncoder.Encoder[id].rx_flag = 0;
	return ENCODER_OK;
}

J_Data_t* get_arm_ptr(void){
	return &Arm;
}

