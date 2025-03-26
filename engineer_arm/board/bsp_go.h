#ifndef BSP_GO_H
#define	BSP_GO_H

#include "struct_typedef.h"


#define REDUCTION_RATIO_GO 6.33f
#define	GO_MOTOR_TX_HEAD1 0xFE 
#define	GO_MOTOR_TX_HEAD2 0xEE 
#define	GO_MOTOR_RX_HEAD1 0xFD 
#define	GO_MOTOR_RX_HEAD2 0xEE 

typedef struct
{
		uint8_t HAED1;
		uint8_t HAED2;
		uint8_t RESERVE;//1
}tx_header;

typedef struct
{
		int16_t T;
		int16_t W;
		int32_t Pos;
		int16_t K_P;
		int16_t K_W;
}tx_message;

typedef struct
{
		uint16_t crc_u;
}CRC16_CHECK_TX;

typedef struct
{		
		tx_header  header;
		tx_message trans_message;
		CRC16_CHECK_TX CRCdata;		
} tx_motor_message;

//rx
typedef struct
{
		uint8_t HAED1;
		uint8_t HAED2;
		uint8_t RESERVE;//1
}rx_header;

typedef struct
{
		int16_t T;
		int16_t W;
		int32_t Pos;
		int8_t Temp;
		int16_t Reserve;//MERROR+Force+RESERVE
}rx_message_temp;

typedef struct
{
		float T;
		float W;
		float Pos;
		float Temp;
		float MERROR;
		float Force;
		float Reserve;
}rx_message;

typedef struct
{
		uint16_t crc_u;
}CRC16_CHECK_RX;

typedef struct
{		
		rx_header  header;
		rx_message trans_message;
		CRC16_CHECK_RX CRCdata;	
} rx_motor_message;


typedef struct
{
		tx_motor_message   motor_send;
		rx_motor_message   motor_recv;
		fp32 speed;
		fp32 angle;//减去初始值的叫angle
		fp32 pos_set;//加上初始值的叫pos
		fp32 offset_ecd;
}motor_GO_t;

extern void GO_motor_position_control(motor_GO_t *GO_motor_position,int i,fp32 pos);
extern void GO_motor_velocity_control(motor_GO_t *GO_motor_velocity,int i,fp32 w);
extern void GO_motor_T_control(motor_GO_t *GO_motor_torque,int i,fp32 torque);
extern void GO_motor_zero_force_control(motor_GO_t *GO_motor_zero_force,int i);
extern void modify_and_send_data_0(void);
extern void modify_and_send_data_1(void);
#endif
