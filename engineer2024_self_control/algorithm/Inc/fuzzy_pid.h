#ifndef __FUZZY_PID_H
#define __FUZZY_PID_H
#include "stdint.h"
#include "arm_math.h"
#define LAST 0
#define NOW 1
#define NB -3.0f
#define NM -2.0f
#define NS -1.0f
#define ZO 0
#define PS 1.0f
#define PM 2.0f
#define PB 3.0f
#define ERR_TO_E 2.0f
#define DELTAERR_TO_EC 1.0f
typedef struct pid_t
{
	float32_t		p;
	float32_t		i;
	float32_t		d;
	float32_t		p_pre;
	float32_t		i_pre;
	float32_t		d_pre;
	float32_t		index_p;
	float32_t		index_i;
	float32_t		index_d;
	float32_t		e;
	float32_t		ec;
	float32_t 		set;
	float32_t 		get;
	float32_t 		err[2];
	float32_t		deltaerr;
	float32_t 		pout;
	float32_t 		iout;
	float32_t 		dout;
	float32_t 		out;
	float32_t 	max_out;
	float32_t 	max_iout;
}fuzzy_pid_t;
float32_t fuzzy_pid_calc(fuzzy_pid_t *pid, float32_t get, float32_t set);
void fuzzy_pid_init(fuzzy_pid_t *pid, float32_t maxout, float32_t maxiout, float32_t kp, float32_t ki, float32_t kd);
#endif
