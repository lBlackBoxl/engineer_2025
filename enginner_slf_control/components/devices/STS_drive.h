#ifndef _SMS_STS_H
#define _SMS_STS_H

#include "main.h"
#include "usart.h"

// 鍐呭瓨琛ㄥ畾涔�
//-------EPROM(鍙�璇�)--------
#define STS_Firmware_master_version 0x0
#define STS_DATA_LENGHT_0 1

#define STS_Firmware_sub_version 0x1
#define STS_DATA_LENGHT_1 1

#define STS_End 0x2
#define STS_DATA_LENGHT_2 1

#define STS_servo_master_version 0x3
#define STS_DATA_LENGHT_3 1

#define STS_servo_sub_version 0x4
#define STS_DATA_LENGHT_4 1

//-------EPROM(璇诲啓)--------
#define STS_ID_ADDR 0x5
#define STS_DATA_LENGHT_5 1

#define STS_BAUD_RATE 0x6
#define STS_DATA_LENGHT_6 1
// 娉㈢壒鐜囧畾涔�
#define BAUD_RATE_1000000 0
#define BAUD_RATE_500000 1
#define BAUD_RATE_250000 2
#define BAUD_RATE_128000 3
#define BAUD_RATE_115200 4
#define BAUD_RATE_76800 5
#define BAUD_RATE_57600 6
#define BAUD_RATE_38400 7

#define STS_RESPONSE_STATUS 0x8 // 鍝嶅簲鐘舵€佸畾涔� 0锛氬彧鏈夎�绘寚浠ゅ拰ping鎸囦护杩斿洖搴旂瓟鍖� 1.瀵规墍鏈夋寚浠よ繑鍥炲簲绛斿寘
#define STS_DATA_LENGHT_8 1

#define STS_MIN_ANGLE_LIMIT 0x9
#define STS_DATA_LENGHT_9 2

#define STS_MAX_ANGLE_LIMIT 0xB
#define STS_DATA_LENGHT_B 2

#define STS_MAX_TEMPERATURE_LIMIT 0xD
#define STS_DATA_LENGHT_D 1

#define STS_MAX_VOLTAGE_LIMIT 0xE
#define STS_DATA_LENGHT_E 1

#define STS_MIN_VOLTAGE_LIMIT 0xF
#define STS_DATA_LENGHT_F 1

#define STS_MAX_TORQUE_LIMIT 0x10
#define STS_DATA_LENGHT_10 2

#define STS_UNLOAD_CONDITION 0x13  //瀵瑰簲浣嶈�剧疆1涓哄紑鍚�鐩稿簲淇濇姢锛屽�瑰簲浣嶇疆0涓哄叧闂�鐩稿簲淇濇姢
#define STS_DATA_LENGHT_13 1

#define STS_ALARM_LED 0x14 //瀵瑰簲浣嶈�剧疆1涓哄紑鍚�闂�鐏�鎶ヨ�︼紝瀵瑰簲浣嶇疆0涓哄叧闂�闂�鐏�鎶ヨ��
#define STS_DATA_LENGHT_14 1

#define STS_PID_KP 0x15  //0~254
#define STS_DATA_LENGHT_15 1

#define STS_PID_KD 0x16  //0~254
#define STS_DATA_LENGHT_16 1

#define STS_PID_KI 0x17  //0~254
#define STS_DATA_LENGHT_17 1

#define STS_MIN_TORQUE_ENABLE 0x18
#define STS_DATA_LENGHT_18 1

#define STS_INTEGRAL_LIMIT 0x19 //鏈€澶хН鍒嗗€�=绉�鍒嗛檺鍒跺€�*4锛�0琛ㄧず鍏抽棴绉�鍒嗛檺鍒跺姛鑳斤紝浣嶇疆妯″紡涓庢�ヨ繘妯″紡鐢熸晥
#define STS_DATA_LENGHT_19 1

#define STS_CLOCKWISE_DEAD_ZONE 0x1A //0~32 鍗曚綅涓烘渶灏忓垎杈ㄨ�掑害锛�0.0879掳锛�
#define STS_DATA_LENGHT_1A 1

#define STS_ANTICLOCKWISE_DEAD_ZONE 0x1B //0~32 鍗曚綅涓烘渶灏忓垎杈ㄨ�掑害锛�0.0879掳锛�
#define STS_DATA_LENGHT_1B 1

#define STS_PROTECTIVE_CURRENT 0x1C //0~511 鍗曚綅涓�6.5mA
#define STS_DATA_LENGHT_1C 2

#define STS_POSITION_CALIBRATION 0x1F //-2047~2047 鍗曚綅涓烘渶灏忓垎杈ㄨ�掑害锛�0.0879掳锛�
#define STS_DATA_LENGHT_1F 2

#define STS_RUNNING_MODE 0x21 //0锛氫綅缃�妯″紡 1锛氶€熷害妯″紡 2锛歅WM寮€鐜�閫熷害妯″紡 3锛氭�ヨ繘妯″紡
#define STS_DATA_LENGHT_21 1
// 杩愯�屾ā寮忓畾涔�
#define STS_POSITION_MODE 0
#define STS_SPEED_MODE 1
#define STS_PWM_MODE 2
#define STS_STEP_MODE 3

#define STS_PROTECTIVE_TORQUE 0x22 //0~100 鍗曚綅涓烘渶澶ф壄鐭╃殑鐧惧垎姣�
#define STS_DATA_LENGHT_22 1

#define STS_PROTECTIVE_TIME 0x23 //0~254 鍗曚綅涓�0.01s
#define STS_DATA_LENGHT_23 1

#define STS_OVERLOAD_TORQUE 0x24 //0~100 鍗曚綅涓烘渶澶ф壄鐭╃殑鐧惧垎姣�
#define STS_DATA_LENGHT_24 1

#define STS_PID_KP_SPEED 0x25 //0~254
#define STS_DATA_LENGHT_25 1

#define STS_OVERLOAD_CURRENT_TIME 0x26 //0~254 鍗曚綅涓�0.01s
#define STS_DATA_LENGHT_26 1

#define STS_PID_KI_SPEED 0x27 //0~254
#define STS_DATA_LENGHT_27 1

//-------SRAM(璇诲啓)--------
#define STS_TORQUE_ENABLE 0x28 // 0锛氬叧闂�鎵�鍔涜緭鍑� 1锛氬紑鍚�鎵�鍔涜緭鍑� 128:浠绘剰褰撳墠浣嶇疆鐭�姝ｄ负2048
#define STS_DATA_LENGHT_28 1

#define STS_ACC_SPEED 0x29 //0~254 鍗曚綅涓�100姝�/绉抆2
#define STS_DATA_LENGHT_29 1

#define STS_TORGET_POSITION 0x2A //-32766 ~ 32766 鍗曚綅涓烘渶灏忓垎杈ㄨ�掑害锛�0.0879掳锛�
#define STS_DATA_LENGHT_2A 2

#define STS_TORGET_RUNNING_TIME 0x2C //0~1000 鍗曚綅涓�0.1%
#define STS_DATA_LENGHT_2C 2

#define STS_TORGET_SPEED 0x2E //-32766 ~ 32766 鍗曚綅涓烘��/绉�
#define STS_DATA_LENGHT_2E 2

#define STS_TORQUE_LIMIT 0x30 //0~100 鍗曚綅涓烘渶澶ф壄鐭╃殑鐧惧垎姣�
#define STS_DATA_LENGHT_30 2

#define STS_LOCK 0x37 //0:鍏抽棴鍐欏叆閿侊紝鍐欏叆鏁版嵁鎺夌數淇濆瓨 1锛氬紑鍚�鍐欏叆閿侊紝鍐欏叆鏁版嵁鎺夌數涓嶄繚瀛�
#define STS_DATA_LENGHT_37 1

//-------SRAM(鍙�璇�)--------
#define STS_PRESENT_POSITION 0x38 //鍗曚綅涓烘��
#define STS_DATA_LENGHT_38 2

#define STS_PRESENT_SPEED 0x3A //鍗曚綅涓烘��/绉�
#define STS_DATA_LENGHT_3A 2

#define STS_PRESENT_LOAD 0x3C //鍗曚綅涓�0.1%,琛ㄧず鐢靛帇鍗犵┖姣�
#define STS_DATA_LENGHT_3C 2

#define STS_PRESENT_VOLTAGE 0x3E //鍗曚綅涓�0.1V
#define STS_DATA_LENGHT_3E 1

#define STS_PRESENT_TEMPERATURE 0x3F //鍗曚綅涓烘憚姘忓害
#define STS_DATA_LENGHT_3F 1

#define STS_ERROR 0x41
#define STS_DATA_LENGHT_41 1

#define STS_PRESENT_MODE 0x42 //鑸垫満杩愬姩鏃朵负1锛屽仠姝㈡椂涓�0
#define STS_DATA_LENGHT_42 1

#define STS_PRESENT_CURRENT 0x45 //鍗曚綅涓�6.5mA
#define STS_DATA_LENGHT_45 2

//鎸囦护绫诲瀷瀹氫箟
#define STS_PING 0x01
#define STS_READ 0x02
#define STS_WRITE 0x03
#define STS_REG_WRITE 0x04
#define STS_ACTION 0x05
#define STS_SYCNREAD 0x82 //鐢ㄤ簬鍚屾椂鏌ヨ�㈠�氫釜鑸垫満
#define STS_SYCNWRITE 0x83 //鐢ㄤ簬鍚屾椂鎺у埗澶氫釜鑸垫満
#define STS_RECOVERY 0x06 //鎭㈠�嶈埖鏈哄嚭鍘傝�剧疆
#define STS_RESET 0x0A //閲嶇疆鑸垫満鍦堟暟
#define STS_POS_ADJ	0x0B//浣嶇疆杈冨噯

//extern uint8_t STS_CMD_PING(UART_HandleTypeDef *huart, uint8_t STS_ID);
//extern uint16_t STS_CMD_READ(UART_HandleTypeDef *huart, uint8_t STS_ID, uint8_t MemAddr, uint8_t rLen);
//extern void STS_CMD_WRITE(UART_HandleTypeDef *huart, uint8_t STS_ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);
//extern void STS_CMD_ASYNC_WRITE(UART_HandleTypeDef *huart, uint8_t STS_ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);
//extern void STS_CMD_ASYNC_ACTION(UART_HandleTypeDef *huart, uint8_t STS_ID);
//extern void STS_CMD_SYNC_WRITE(UART_HandleTypeDef *huart, uint8_t STS_ID[], uint8_t IDN, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);
//extern uint8_t STS_CMD_SYNC_READ(UART_HandleTypeDef *huart, uint8_t STS_ID[], uint8_t IDN, uint8_t MemAddr, uint8_t *rData[], uint8_t rLen);
//extern uint8_t STS_CMD_RECOVERY(UART_HandleTypeDef *huart, uint8_t STS_ID);
//extern uint8_t STS_CMD_RESET(UART_HandleTypeDef *huart, uint8_t STS_ID);
//extern uint8_t STS_CMD_POS_ADJ(UART_HandleTypeDef *huart, uint8_t STS_ID, uint16_t target_position);

#endif
