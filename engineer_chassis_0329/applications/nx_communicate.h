#ifndef NX_COMMUNICATE_H
#define NX_COMMUNICATE_H

#include "main.h"
#include "struct_typedef.h"

#define SOF_ADDR 	 0			//֡ͷ����ʽ�ֽ�ƫ����

#define FRAME_HEADER 		0x69//֡ͷ����ʽ
#define LEN_FRAME_HEADER 	1	//֡ͷ����
#define	LEN_TX_DATA 		24	//�������ݶγ���
#define	LEN_RX_DATA 		24	//�������ݶγ���
#define	LEN_FRAME_TAILER 	2	//֡βCRC16
#define	LEN_TX_PACKET		27	//���Ͱ���������
#define	LEN_RX_PACKET		2	//���հ���������

#define USART7_MAX_RECV_LEN 100

void nx_communicate_task(void const *pvParameters);

typedef __packed struct
{
	uint8_t  	sof;			// ֡ͷ����ʽ
} frame_header_t;
//֡β�ṹ��
typedef __packed struct 
{
	uint8_t crc16;				// CRC16У����
} frame_tailer_t;

//�������ݽṹ��
typedef __packed struct 
{
	fp32 motor1_position;
	fp32 motor2_position;
	fp32 motor3_position;
	fp32 motor4_position;
	fp32 motor5_position;
	fp32 motor6_position;
}tx_data_t;

//�������ݽṹ��
typedef __packed struct 
{
	fp32 motor1_position;
	fp32 motor2_position;
	fp32 motor3_position;
	fp32 motor4_position;
	fp32 motor5_position;
	fp32 motor6_position;
}rx_data_t;

//���Ͱ��ṹ��
typedef __packed struct 
{
	frame_header_t 		frame_header;	
	tx_data_t	  		tx_data;	
	uint8_t					robot_id;
	frame_tailer_t 		frame_tailer;	
} send_msg_t;

//���ܰ��ṹ��
typedef __packed struct 
{
	frame_header_t	 	frame_header;	
	rx_data_t	  		rx_data;	
	frame_tailer_t 		frame_tailer;	
} receive_msg_t;

extern send_msg_t pc_send_msg;
extern receive_msg_t pc_receive_msg;
extern float nx_allowance[6];
#endif
