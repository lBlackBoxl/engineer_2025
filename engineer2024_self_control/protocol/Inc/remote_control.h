#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "stdint.h"

#define SBUS_RX_BUF_NUM 36u //缓冲区大小（防止数据越界的较大长度）
#define RC_FRAME_LENGTH 18u //缓冲区大小（实际接收长度）

#define RC_CH_VALUE_MIN         ((uint16_t)364) //遥控器通道最小值
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)//遥控器通道偏移量
#define RC_CH_VALUE_MAX         ((uint16_t)1684)//遥控器通道最大值

#define RC_SW_UP                ((uint16_t)1)       //拨杆上值
#define RC_SW_MID               ((uint16_t)3)       //拨杆中值
#define RC_SW_DOWN              ((uint16_t)2)       //拨杆下值
#define switch_is_down(s)       (s == RC_SW_DOWN)   //拨杆打到下
#define switch_is_mid(s)        (s == RC_SW_MID)    //拨杆打到中
#define switch_is_up(s)         (s == RC_SW_UP)     //拨杆打到上

#define KEY_PRESSED_OFFSET_W        ((uint16_t)1 << 0)  //W偏移量
#define KEY_PRESSED_OFFSET_S        ((uint16_t)1 << 1)  //A偏移量
#define KEY_PRESSED_OFFSET_A        ((uint16_t)1 << 2)  //S偏移量
#define KEY_PRESSED_OFFSET_D        ((uint16_t)1 << 3)  //D偏移量
#define KEY_PRESSED_OFFSET_SHIFT    ((uint16_t)1 << 4)  //SHIFT偏移量
#define KEY_PRESSED_OFFSET_CTRL     ((uint16_t)1 << 5)  //CTRL偏移量
#define KEY_PRESSED_OFFSET_Q        ((uint16_t)1 << 6)  //Q偏移量
#define KEY_PRESSED_OFFSET_E        ((uint16_t)1 << 7)  //E偏移量
#define KEY_PRESSED_OFFSET_R        ((uint16_t)1 << 8)  //R偏移量
#define KEY_PRESSED_OFFSET_F        ((uint16_t)1 << 9)  //F偏移量
#define KEY_PRESSED_OFFSET_G        ((uint16_t)1 << 10) //G偏移量
#define KEY_PRESSED_OFFSET_Z        ((uint16_t)1 << 11) //Z偏移量
#define KEY_PRESSED_OFFSET_X        ((uint16_t)1 << 12) //X偏移量
#define KEY_PRESSED_OFFSET_C        ((uint16_t)1 << 13) //C偏移量
#define KEY_PRESSED_OFFSET_V        ((uint16_t)1 << 14) //V偏移量
#define KEY_PRESSED_OFFSET_B        ((uint16_t)1 << 15) //B偏移量

typedef struct          //遥控器数据结构体
{
    struct              //遥控器值
    {
        int16_t ch[5];  //通道值
        char s[2];      //拨杆值
    } rc;
    struct              //鼠标值
    {
        int16_t x;      //左右值
        int16_t y;      //前后值
        int16_t z;      //滚轮值
        uint8_t press_l;//左键值
        uint8_t press_r;//右键值
    } mouse;
    struct              //按键值
    {
        uint16_t v;     //按键状态值
    } key;
}rc_ctrl_t;

//遥控器结构体
extern rc_ctrl_t rc_ctrl;

/// @brief 遥控器初始化
/// @param  无
extern void remote_control_init(void);

/// @brief 获取遥控器结构体指针
/// @param  无
/// @return 遥控器结构体
extern const rc_ctrl_t *get_remote_control_point(void);

#endif
