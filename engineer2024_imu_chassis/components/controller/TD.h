#ifndef TD_H
#define TD_H
#include "struct_typedef.h"

typedef struct
{
		fp32 h;
	  fp32 h0;
		fp32 N0;
		fp32 r;
	  fp32 fhan;
		fp32 ddx;
	  fp32 dx;
		fp32 x;
} TD_t;

extern void TD_init(TD_t *td,float r,float N0,float h,float x);
extern float TD_calc(TD_t *td,float v);
#endif


