#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include "struct_typedef.h"
#include <string.h>


#define ENCODER_RESOLUTION  12

typedef enum{
	ANGH	=		0x0040,
	ANGL = 		0x0041,
	VELH = 		0x0042,
	VELL	=		0x0043,
	CYC  =		0x0044,
	MCYCH =		0x0046,
	MCYCL =		0x0047,
	
	MID = 		0x0051,
	ZERO = 		0x0052,
	DIR	=			0x0006,
	LOCK =	  0x0055,
	BALD = 		0x0000,
	ID	 =    0x0005,
	
	RST = 		0x0054,
	FAC_RST = 0x0053,

	TEST_REG = 0x01
}Encoder_Reg_t;

typedef enum{
	RD_REG = 0x03,
	WRT_REG= 0x06
}Encoder_Cmd_t;

typedef enum{
	ENCODER_OK = 0x00,
	ENCODER_ERROR = 0x01,
	ENCODER_BUSY = 0x02,
}Encoder_Status_e;

typedef enum{
	READ_ANGLE = 0x00,
	READ_CYCLE = 0x01,
	READ_MCYCLE = 0x02,
	DISABLE_READ,
}Encoder_Tx_Mode_e;

typedef __packed struct{
	uint8_t		id ;
	uint8_t		cmd_t;		
	uint16_t	reg;
	uint16_t  data;
	uint16_t	crc16;
}Encoder_Tx_t;

typedef __packed struct{
	__packed union{
		Encoder_Tx_t data;
		uint8_t hex[8];
	}encoder;
}Tx_t;

typedef __packed struct{
	uint8_t id;
	uint8_t cmd_t;
	uint8_t data_len;
}Encoder_RxHead_t;

typedef struct{
	uint16_t data16;
	uint32_t data32;
	fp32		 fpData;
}Encoder_Angle_t;

typedef struct{
	uint32_t 	raw;
	fp32		 	angle;
}Encoder_Mcycle_t;

typedef struct{
	uint16_t cycle;
	Encoder_Angle_t angle;
}Encoder_RxData_t;

typedef struct{
	Encoder_RxHead_t header;
	Encoder_RxData_t data;
	uint8_t rx_flag;
}Encoder_Rx_t;

typedef struct{
	uint8_t TxMode;
	Tx_t Sender;
	Encoder_Rx_t Encoder[7];
}Encoder_t;

//地址偏移量（该宏定义应等于最小的id值）
#ifndef ADDRESS_OFFSET
#define ADDRESS_OFFSET 1 
#endif

extern Encoder_t AllEncoder;

extern void read_angle_init(Tx_t* pData, uint8_t id);
extern void read_cycle_init(Tx_t* pData, uint8_t id);
extern void read_cycle_angle_init(Tx_t* pData, uint8_t id);

extern void set_zerop_init(Tx_t* pData, uint8_t id);
extern void set_midp_init(Tx_t* pData, uint8_t id);
extern void encoder_reset_init(Tx_t* pData, uint8_t id);
extern void encoder_buf_init(void);
extern Encoder_Status_e unpack_encoder_data(uint8_t* pData);

#endif
