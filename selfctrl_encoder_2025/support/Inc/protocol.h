#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H
#include "struct_typedef.h"

#ifndef USER_NULL
#define USER_NULL (void*)0
#endif

#define REF_PROTOCOL_FRAME_MAX_SIZE         128
#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))
#pragma pack(push, 1)
typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;
typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;
typedef struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;
#pragma pack(pop)
extern uint8_t get_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength, uint8_t ucCRC8);
extern uint8_t verify_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
extern void append_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
extern uint16_t get_crc16_check_sum_referee(uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC);
extern uint8_t verify_crc16_check_sum_referee(uint8_t* pchMessage, uint32_t dwLength);
extern uint8_t verify_crc16_check_sum_encoder(uint8_t* pchMessage, uint32_t dwLength);
extern void append_crc16_check_sum_referee(uint8_t* pchMessage, uint32_t dwLength);
extern void append_crc16_check_sum_encoder(uint8_t* pchMessage, uint32_t dwLength);
#endif
