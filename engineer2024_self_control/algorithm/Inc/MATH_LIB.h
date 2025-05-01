#ifndef MATH_LIB_H
#define MATH_LIB_H

#include "stdint.h"
#include "arm_math.h"

typedef struct
{
    float32_t slope;    //输入
    float32_t out;      //输出
    float32_t min_value;//最小值
    float32_t max_value;//最大值
    float32_t dt;       //步长
} ramp_t;               //斜坡函数结构体

//最小二乘拟合
typedef __packed struct
{
    uint16_t Order;
    uint32_t Count;

    float *x;
    float *y;

    float k;
    float b;

    float StandardDeviation;

    float t[4];
} Ordinary_Least_Squares_t;
/// @brief 快速开方
/// @param x 原数据
/// @return 开方结果
extern float quick_sqrt(float x);

/// @brief 斜坡函数计算
/// @param ramp 斜坡函数结构体
/// @param slope 斜率
extern void ramp_calc(ramp_t *ramp, float32_t slope);

/// @brief 取绝对值
/// @param input 原数据
/// @return 正值数据
extern float32_t float32_abs(float32_t input);

/// @brief 限幅函数
/// @param input 输入数据
/// @param min 最小值
/// @param max 最大值
/// @return 限幅后结果
extern float32_t float32_limit(float32_t input, float32_t min, float32_t max);

/// @brief 弧度限幅函数（限制在±PI之间）
/// @param input 原弧度
/// @return 限幅函数
extern float32_t rad_format(float32_t input);
extern float sin_calc(float amplitude, float frequency, float Ts);
extern void ramp_init(ramp_t *ramp_source_type, float frame_period, float max, float min);
void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order);
void OLS_Update(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float Get_OLS_Derivative(Ordinary_Least_Squares_t *OLS);
float Get_OLS_Smooth(Ordinary_Least_Squares_t *OLS);
#endif
