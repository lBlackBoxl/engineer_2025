#ifndef PC_SPEED_H
#define PC_SPEED_H
#include "pc_auto_aim.h"


#define	LEN_VEL_RX_PACKET		17	//接收包整包长度
#define LEN_IMU_TX_PACKET       41

typedef struct 
{
	int16_t is_continue;   
	int16_t   vx;
	int16_t   vy;
	float32_t   vw;
}vel_rx_data_t;


typedef __packed struct
{
	uint8_t sof;
	float32_t q0;
	float32_t q1;
	float32_t q2;
	float32_t q3;
	
	float32_t gyro_x;
	float32_t gyro_y;
	float32_t gyro_z;
	
	float32_t accel_x;
	float32_t accel_y;
	float32_t accel_z;
	
	uint8_t end;
}imu_data_t;



typedef struct 
{
	//frame_header_t	 	frame_header;	
	vel_rx_data_t	  	vel_rx_data;	
//	frame_tailer_t 		frame_tailer;	
} vel_receive_msg_t;

extern void pc_send_imu_task(void const *argu);
extern vel_rx_data_t vel_ctrl;
extern vel_rx_data_t *get_pc_vel(void);
extern uint64_t run_dog;
#endif



