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
extern void TD_set_x(TD_t *td,float x);
extern void TD_set_r(TD_t *td,float r);
extern float TD_calc(TD_t *td,float v);
extern float TD_calc_angle(TD_t *td,float v);

#endif


