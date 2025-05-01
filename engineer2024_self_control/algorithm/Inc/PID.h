#ifndef PID_H
#define PID_H

#include "stdint.h"
#include "arm_math.h"

typedef enum            //PID模式
{
    PID_POSITION = 0,   //位置式PID
    PID_DELTA           //增量式PID
} pid_mode_e;           

typedef struct          //PID结构体
{
    pid_mode_e mode;    //PID模式
    float32_t Kp;       //比例系数
    float32_t Ki;       //积分系数
    float32_t Kd;       //微分系数
    float32_t max_out;  //最大输出
    float32_t max_iout; //最大积分项输出
    float32_t set;      //设定值
    float32_t fdb;      //反馈值
    float32_t out;      //输出
    float32_t Pout;     //比例项输出
    float32_t Iout;     //积分项输出
    float32_t Dout;     //微分项输出
    float32_t Dbuf[3];  //最新的三个微分项
    float32_t error[3]; //最新的三个误差项
    float32_t err;      //速度环、角度环pid用
    float32_t get;      //速度环、角度环pid用
} pid_type_def;         

/// @brief PID初始化
/// @param pid PID结构体
/// @param mode PID模式
/// @param kp 比例系数
/// @param ki 积分系数
/// @param kd 微分系数
/// @param max_out 最大输出
/// @param max_iout 最大积分项输出
extern void PID_init(pid_type_def *pid, uint8_t mode,float32_t kp,float32_t ki,float32_t kd, float32_t max_out, float32_t max_iout);

/// @brief 角度环PID
/// @param pid PID结构体
/// @param fdb PID反馈值
/// @param set PID设定值
/// @return PID输出值
extern float32_t PID_calc_angle(pid_type_def *pid, float32_t fdb, float32_t set);

/// @brief 一般PID
/// @param pid PID结构体
/// @param fdb PID反馈值
/// @param set PID设定值
/// @return PID输出值 
extern float32_t PID_calc(pid_type_def *pid, float32_t fdb, float32_t set);

extern void gimbal_PID_init(pid_type_def *pid, float32_t maxout, float32_t max_iout, float32_t kp, float32_t ki, float32_t kd);
extern float32_t gimbal_PID_calc(pid_type_def *pid, float32_t get, float32_t set, float32_t error_delta);
extern void gimbal_PID_clear(pid_type_def *gimbal_pid_clear);
#endif
