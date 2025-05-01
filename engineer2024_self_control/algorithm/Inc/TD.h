#ifndef TD_H
#define TD_H
#include "stdint.h"
#include "arm_math.h"

typedef struct
{
	float32_t h;
	float32_t h0;
	float32_t N0;
	float32_t r;
	float32_t fhan;
	float32_t ddx;
	float32_t dx;
	float32_t x;
} TD_t;

extern void TD_init(TD_t *td,float r,float N0,float h,float x);
extern float TD_calc(TD_t *td,float v);
#endif


