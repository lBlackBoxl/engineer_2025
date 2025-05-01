#include "fuzzy_pid.h"
#include "MATH_LIB.h"
//E和EC的数组，可以看到它们存放的数据相同，这说明在使用E和EC前需要对error和deltaerror统一量纲，参数需要手调
float32_t E[7] = {NB,NM,NS,ZO,PS,PM,PB};
float32_t EC[7] = {NB,NM,NS,ZO,PS,PM,PB};
//p,i,d共三个规则表
//每一个7*7矩阵，存放规则表的隶属度值，参数需要手调
float32_t rule_p[7][7] =	{PB, PB, PM, PM, PS, ZO, ZO,
							 PB, PB, PM, PS, PS, ZO, NS,		
							 PM, PM, PM, PS, ZO, NS, NS,
							 PM, PM, PS, ZO, NS, NM, NM,
							 PS, PS, ZO, NS, NS, NM, NM,
							 PS, ZO, NS, NM, NM, NM, NB,
							 ZO, ZO, NM, NM, NM, NB, NB,};

float32_t rule_i[7][7] =	{NB, NB, NM, NM, NS, ZO, ZO,
							 NB, NB, NM, NS, NS, ZO, ZO, 			
							 NB, NM, NS, NS, ZO, PS, PS,
							 NM, NM, NS, ZO, PS, PM, PM,
							 NM, NS, ZO, PS, PS, PM, PB,
							 ZO, ZO, PS, PS, PM, PB, PB,
							 ZO, ZO, PS, PM, PM, PB, PB,};

float32_t rule_d[7][7] =	{PS, NS, NB, NB, NB, NM, PS,
							 PS, NS, NB, NM, NM, NS, ZO, 			
							 ZO, NS, NM, NM, NS, NS, ZO,
							 ZO, NS, NS, NS, NS, NS, ZO,
							 ZO, ZO, ZO, ZO, ZO, ZO, ZO,
							 PB, NS, PS, PS, PS, PS, PB,
							 PB, PM, PM, PM, PS, PS, PB,};
//最大值限制
void abs_limit(float32_t *out, float32_t ABS_MAX)
{
	if(*out > ABS_MAX)
		*out = ABS_MAX;
	else if(*out < -ABS_MAX)
		*out = -ABS_MAX;
}
//清晰化
void fuzzy_assignment(fuzzy_pid_t *pid)
{
	uint8_t i;
	uint8_t index_e, index_ec;
	float32_t index_p, index_i, index_d;
	float32_t alpha_p, beta_p;
	float32_t alpha_i, beta_i;
	float32_t alpha_d, beta_d;
	//对error和deltaerror统一量纲，其中ERR_TO_E和DELTAERR_TO_EC参数手调
	pid->e = pid->err[NOW] / ERR_TO_E;
	pid->ec = (pid->err[NOW] - pid->err[LAST]) / DELTAERR_TO_EC;
	if(pid->e < E[0])
		pid->e = E[0];
	if(pid->e > E[6])
		pid->e = E[6];
	if(pid->ec< EC[0])
		pid->ec = EC[0];
	if(pid->ec > EC[6])
		pid->ec = EC[6];
	//查询并记录error在E中处于什么区域，以及deltaerror在EC中处于什么区域
	for(i = 0; i < 6; i++)
	{
		if(pid->e >= E[i])
			index_e = i;
	}
	for(i = 0; i< 6; i++)
	{
		if(pid->ec >= EC[i])
			index_ec = i;
	}
	//计算p,i,d隶属度值并最终输出p,i,d参数
	alpha_p = (pid->e - rule_p[index_e][index_ec]) / (rule_p[index_e + 1][index_ec] - rule_p[index_e][index_ec]);
	beta_p =  (pid->ec - rule_p[index_e][index_ec]) / (rule_p[index_e][index_ec + 1] - rule_p[index_e][index_ec]);
	alpha_i = (pid->e - rule_i[index_e][index_ec]) / (rule_i[index_e + 1][index_ec] - rule_i[index_e][index_ec]);
	beta_i =  (pid->ec - rule_i[index_e][index_ec]) / (rule_i[index_e][index_ec + 1] - rule_i[index_e][index_ec]);
	alpha_d = (pid->e - rule_d[index_e][index_ec]) / (rule_d[index_e + 1][index_ec] - rule_d[index_e][index_ec]);
	beta_d =  (pid->ec - rule_d[index_e][index_ec]) / (rule_d[index_e][index_ec + 1] - rule_d[index_e][index_ec]);
	pid->index_p = rule_p[index_e][index_ec] * alpha_p * beta_p 
				 + rule_p[index_e][index_ec + 1] * alpha_p * (1.0f - beta_p)
				 + rule_p[index_e + 1][index_ec] * (1.0f - alpha_p) * beta_p
				 + rule_p[index_e + 1][index_ec + 1] * (1.0f - alpha_p) * (1.0f - beta_p);
	pid->index_i = rule_i[index_e][index_ec] * alpha_i * beta_i 
				 + rule_i[index_e][index_ec + 1] * alpha_i * (1.0f - beta_i)
				 + rule_i[index_e + 1][index_ec] * (1.0f - alpha_i) * beta_i
				 + rule_i[index_e + 1][index_ec + 1] * (1.0f - alpha_i) * (1.0f - beta_i);
	pid->index_d = rule_d[index_e][index_ec] * alpha_d * beta_d 
				 + rule_d[index_e][index_ec + 1] * alpha_d * (1.0f - beta_d)
				 + rule_d[index_e + 1][index_ec] * (1.0f - alpha_d) * beta_d
				 + rule_d[index_e + 1][index_ec + 1] * (1.0f - alpha_d) * (1.0f - beta_d); 
	pid->p = pid->p_pre * index_p;
	pid->i = pid->i_pre * index_i;
	pid->d = pid->d_pre * index_d;
}
//pid结构体初始化
void fuzzy_pid_init(fuzzy_pid_t *pid, float32_t maxout, float32_t maxiout,float32_t kp, float32_t ki, float32_t kd)
{
	pid->max_out 	= maxout;
	pid->max_iout 	= maxiout;
	pid->p			= kp;
	pid->i			= ki;	
	pid->d			= kd;
	pid->p_pre		= kp;
	pid->i_pre		= ki;
	pid->d_pre		= kd;	
	pid->pout 		= 0;
	pid->iout 		= 0;
	pid->dout 		= 0;
	pid->out 		= 0;
}
//pid计算
float32_t fuzzy_pid_calc(fuzzy_pid_t *pid, float32_t get, float32_t set)
{
	pid->get = get;
	pid->set = set;
	pid->err[NOW] = set - get;
	fuzzy_assignment(pid);
	pid->pout = pid->p * pid->err[NOW];
	pid->iout += pid->i * pid->err[NOW];
	abs_limit(&(pid->iout), pid->max_iout);
	pid->dout= pid->d * (pid->err[NOW] - pid->err[LAST]);
	pid->out = pid->pout + pid->iout + pid->dout;
	abs_limit(&(pid->out), pid->max_out);
	pid->err[LAST] = pid->err[NOW];
	return pid->out;
}
