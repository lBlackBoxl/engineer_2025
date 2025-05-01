#ifndef Butterworth_Filter_H
#define Butterworth_Filter_H
#include "struct_typedef.h"


typedef struct
{
	fp32 Fc;
	fp32 Ts;
	fp32 Wc;
	fp32 Dp;
	fp32 b0;
	fp32 a0;
	fp32 a1;
	fp32 a2;	
	fp32 Xin[3];
	fp32 Yout[3];
} Butter_worth_Filter_t;

extern void Butter_worth_Filter_init(Butter_worth_Filter_t *Filter,fp32 Fc,fp32 Ts);
extern fp32 Butter_worth_Filter_calc(Butter_worth_Filter_t *Filter,fp32 xin);
#endif

