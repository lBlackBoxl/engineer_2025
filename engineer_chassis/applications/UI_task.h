#ifndef UI_TASK_H
#define UI_TASK_H

#include <stdio.h>
#include "ui_types.h"
#include "usart.h"

extern int ui_self_id;
extern const uint16_t wCRC_Table[256];
extern const uint8_t CRC8_TAB[256];

void UI_task(void const *pvParameters);
void print_message(const uint8_t* message, int length);

#define SEND_MESSAGE(message, len) HAL_UART_Transmit(&huart6, message, len, 0xffff);

void ui_proc_1_frame(ui_1_frame_t *msg);
void ui_proc_2_frame(ui_2_frame_t *msg);
void ui_proc_5_frame(ui_5_frame_t *msg);
void ui_proc_7_frame(ui_7_frame_t *msg);
void ui_proc_string_frame(ui_string_frame_t *msg);

#endif

