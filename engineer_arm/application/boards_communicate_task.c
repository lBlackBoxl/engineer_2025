#include "boards_communicate_task.h"
#include "can_communicate.h"
#include "arm_control_task.h"
#include "cmsis_os.h"

static void TX_code(void);

Tx_Message_t tx_message_mp;
Rx_Message_t rx_message_mp;

void A_communicate_task(void const * argument)
{
	while(1)
	{
		TX_code();
		CAN_board_communicate_can2_0(tx_message_mp.motor_position);
		osDelay(1);
		CAN_board_communicate_can2_1(tx_message_mp.motor_position);
		osDelay(1);
	}
}

void TX_code(void)
{
	tx_message_mp.motor_position[1] = arm.motor_DM_data[0].DM_motor_measure->motor_position;
	tx_message_mp.motor_position[2] = arm.motor_DM_data[1].DM_motor_measure->motor_position;
	tx_message_mp.motor_position[3] = arm.motor_DM_data[2].DM_motor_measure->motor_position;
	tx_message_mp.motor_position[4] = arm.roll_angle;//转化成末端两轴角度
	tx_message_mp.motor_position[5] = arm.yaw_angle;
}
