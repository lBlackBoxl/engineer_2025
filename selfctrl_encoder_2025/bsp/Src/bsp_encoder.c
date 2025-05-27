#include "bsp_encoder.h"
#include "protocol.h"

Encoder_t AllEncoder;


void read_angle_init(Tx_t* pData, uint8_t id){
	AllEncoder.TxMode = READ_ANGLE;
	pData->encoder.hex[0] = id;
	pData->encoder.hex[1] = RD_REG;
	pData->encoder.hex[3] = (uint8_t)(ANGL<<8);
	pData->encoder.hex[3] = (uint8_t)(ANGL);
	pData->encoder.hex[4] = 0x00;
	pData->encoder.hex[5] = 0x01;
	append_crc16_check_sum_encoder(pData->encoder.hex, sizeof(uint8_t)*8);
}

void read_cycle_init(Tx_t* pData, uint8_t id){
	AllEncoder.TxMode = READ_CYCLE;
	pData->encoder.hex[0] = id;
	pData->encoder.hex[1] = RD_REG;
	pData->encoder.hex[3] = (uint8_t)(CYC<<8);
	pData->encoder.hex[3] = (uint8_t)(CYC);
	pData->encoder.hex[4] = 0x00;
	pData->encoder.hex[5] = 0x01;
	append_crc16_check_sum_encoder(pData->encoder.hex, sizeof(uint8_t)*8);
}

void read_cycle_angle_init(Tx_t* pData, uint8_t id){
	AllEncoder.TxMode = READ_CYCLE;
	pData->encoder.hex[0] = id;
	pData->encoder.hex[1] = RD_REG;
	pData->encoder.hex[3] = (uint8_t)(MCYCH<<8);
	pData->encoder.hex[3] = (uint8_t)(MCYCH);
	pData->encoder.hex[4] = 0x00;
	pData->encoder.hex[5] = 0x02;
	append_crc16_check_sum_encoder(pData->encoder.hex, sizeof(uint8_t)*8);
}

void set_zerop_init(Tx_t* pData, uint8_t id){
	AllEncoder.TxMode = DISABLE_READ;
	pData->encoder.hex[0] = id;
	pData->encoder.hex[1] = WRT_REG;
	pData->encoder.hex[3] = (uint8_t)(ZERO<<8);
	pData->encoder.hex[3] = (uint8_t)(ZERO);
	pData->encoder.hex[4] = 0x00;
	pData->encoder.hex[5] = 0x01;
	append_crc16_check_sum_encoder(pData->encoder.hex, sizeof(uint8_t)*8);
}

void set_midp_init(Tx_t* pData, uint8_t id){
	AllEncoder.TxMode = DISABLE_READ;
	pData->encoder.hex[0] = id;
	pData->encoder.hex[1] = WRT_REG;
	pData->encoder.hex[3] = (uint8_t)(MID<<8);
	pData->encoder.hex[3] = (uint8_t)(MID);
	pData->encoder.hex[4] = 0x00;
	pData->encoder.hex[5] = 0x01;
	append_crc16_check_sum_encoder(pData->encoder.hex, sizeof(uint8_t)*8);
}

void encoder_reset_init(Tx_t* pData, uint8_t id){
	AllEncoder.TxMode = READ_ANGLE;
	pData->encoder.hex[0] = id;
	pData->encoder.hex[1] = WRT_REG;
	pData->encoder.hex[2] = (uint8_t)(RST<<8);
	pData->encoder.hex[3] = (uint8_t)(RST);
	pData->encoder.hex[4] = 0x00;
	pData->encoder.hex[5] = 0x01;
	append_crc16_check_sum_encoder(pData->encoder.hex, sizeof(uint8_t)*8);
}

void encoder_buf_init(void){
	uint8_t i;
	for(i=0;i<8;i++){
		memset(&AllEncoder.Encoder[i], 0, sizeof(Encoder_Rx_t));
	}
}



Encoder_Status_e unpack_encoder_data(uint8_t* pData){
	if(pData[1] == 0x03){
		memcpy(&AllEncoder.Encoder[pData[0] - ADDRESS_OFFSET].header.id, pData, 3*sizeof(uint8_t));
		switch(AllEncoder.TxMode){
			case READ_ANGLE:{
				switch(AllEncoder.Encoder[pData[0] - ADDRESS_OFFSET].header.data_len){
					case 0x02:{
						if(verify_crc16_check_sum_encoder(pData, 7)){
							AllEncoder.Encoder[pData[0] - ADDRESS_OFFSET].data.angle.data16 = (uint16_t)(pData[3]<<8) | (pData[4]);
							AllEncoder.Encoder[pData[0] - ADDRESS_OFFSET].data.angle.fpData = 360.0f*AllEncoder.Encoder[pData[0] - ADDRESS_OFFSET].data.angle.data16 / (1 << ENCODER_RESOLUTION);
							AllEncoder.Encoder[pData[0] - ADDRESS_OFFSET].rx_flag = 1;
							return ENCODER_OK;
						}
						else{
							AllEncoder.Encoder[pData[0] - ADDRESS_OFFSET].rx_flag = 0;
							return ENCODER_ERROR;
						}
					}
					case 0x04:{
						if(verify_crc16_check_sum_encoder(pData, 9)){
							AllEncoder.Encoder[pData[0] - ADDRESS_OFFSET].data.angle.data32 = (uint32_t)(pData[3]<<24) | (pData[4]<<16) | (pData[5]<<8) | (pData[6]);
							AllEncoder.Encoder[pData[0] - ADDRESS_OFFSET].data.angle.fpData = 360.0f*AllEncoder.Encoder[pData[0] - ADDRESS_OFFSET].data.angle.data32 / (1 << ENCODER_RESOLUTION);
							AllEncoder.Encoder[pData[0] - ADDRESS_OFFSET].rx_flag = 1;
							return ENCODER_OK;
						}
						else{
							AllEncoder.Encoder[pData[0] - ADDRESS_OFFSET].rx_flag = 0;
							return ENCODER_ERROR;
						}
					}
					default:{
						AllEncoder.Encoder[pData[0] - ADDRESS_OFFSET].rx_flag = 0;
						return ENCODER_ERROR;
					}
				}
			}
			case READ_CYCLE:{
				if(verify_crc16_check_sum_encoder(pData, 7)){
					AllEncoder.Encoder[pData[0] - ADDRESS_OFFSET].data.cycle = (uint16_t)(pData[3]<<8) | (pData[4]);
					AllEncoder.Encoder[pData[0] - ADDRESS_OFFSET].rx_flag = 1;
					return ENCODER_OK;
				}
				else {
					AllEncoder.Encoder[pData[0] - ADDRESS_OFFSET].rx_flag = 0;
					return ENCODER_ERROR;
				}
			}
			case DISABLE_READ:{//²»Ð£Ñé
				return ENCODER_OK;
			}
			default :{
				AllEncoder.Encoder[pData[0] - ADDRESS_OFFSET].rx_flag = 0;
				return ENCODER_ERROR;
			}
		}
	}
	else if(pData[1] == 0x06){
		if(verify_crc16_check_sum_encoder(pData, 8)){
			return ENCODER_OK;
		}
		else{
			return ENCODER_ERROR;
		}
	}
	else{
		return ENCODER_BUSY;
	}
}
