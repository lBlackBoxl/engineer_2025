#ifndef REFEREE_USART_TASK_H
#define REFEREE_USART_TASK_H

#include "main.h"
#include "referee.h"
#include "struct_typedef.h"
#include "bsp_rc.h"
#include "keyboard.h"
#include "app_config.h"

#define USART_RX_BUF_LENGHT     512
#define REFEREE_FIFO_BUF_LENGTH 1024

#define SBUS_RX_BUF_NUM 42u

#define RC_FRAME_LENGTH 21u

#define SELF_CONTROL_FRAME_LENGTH 39u

#define RC_FRAME_1 0xA9
#define RC_FRAME_2 0x53

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
//#define RC_SW_UP                ((uint16_t)1)
//#define RC_SW_MID               ((uint16_t)3)
//#define RC_SW_DOWN              ((uint16_t)2)
//#define switch_is_down(s)       (s == RC_SW_DOWN)
//#define switch_is_mid(s)        (s == RC_SW_MID)
//#define switch_is_up(s)         (s == RC_SW_UP)

#define RC_SW_RIGHT                ((uint16_t)3)
#define RC_SW_MID               ((uint16_t)1)
#define RC_SW_LEFT              ((uint16_t)2)
#define switch_is_left(s)       (s == RC_SW_LEFT)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_right(s)         (s == RC_SW_RIGHT)



typedef __packed struct
{
				__packed struct
        {
                uint8_t frame[2];
        } header;

        __packed struct
        {
                int16_t ch[5];
                char s[5];
        } rc;
				
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
								uint8_t press_mid;
        } mouse;
        __packed struct
        {
//                uint16_t v;
								bool_t W;
								bool_t S;
								bool_t A;
								bool_t D;
								bool_t SHIFT;
								bool_t CTRL;
								bool_t Q;
								bool_t E;
								bool_t R;
								bool_t F;
								bool_t G;
								bool_t Z;
								bool_t X;
								bool_t C;
								bool_t V;
								bool_t B;
        } key;
				__packed struct
        {
                uint16_t crc_data;
        } CRC16;

} RC_ctrl_t;

extern RC_ctrl_t rc_ctrl;

extern void referee_usart_task(void const * argument);
extern void USART6_IRQHandler(void);
extern key_t last_s[5];
extern void serial2_selfcontrol_rc_init(void);
extern const RC_ctrl_t *get_remote_control_point(void);
extern uint8_t RC_data_is_error(void);
extern void slove_RC_lost(void);
extern void slove_data_error(void);
void usart1_IRQHandler(void);

#endif
