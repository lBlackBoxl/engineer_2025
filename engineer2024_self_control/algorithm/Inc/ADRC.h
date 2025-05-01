#ifndef ADRC_H
#define ADRC_H
#include "stdint.h"
#include "arm_math.h"

typedef struct			//一阶LADRC的参数（不变）结构体
{		
	float32_t Beita;   	//离散化的LESO带宽β，β=e^-woh
	float32_t Beita_2; 	//Beita_2=Beita*Beita
	float32_t wo;		//LESO带宽
	float32_t wc;		//控制器带宽
	float32_t b;       	//控制器增益系数	
	float32_t h;  		//步长（任务周期）
	float32_t max_out;	//最大输出限幅
} ADRC_1_parameter_t;

typedef struct			//一阶LADRC的信号（随时间改变）结构体
{		
    float32_t ref;		//设定值
    float32_t w;  		//云台角速度
	float32_t u;      	//给系统的输入or控制器输出
	float32_t z[2];   	//LESO的状态变量的观测值 
	float32_t y[2];   	//LESO观测器输出
} ADRC_1_singal_t;

typedef struct			//一阶LADRC
{			
	ADRC_1_parameter_t parameter;
	ADRC_1_singal_t singal;
} ADRC_1_t;

extern void ADRC_1_Init(ADRC_1_t *adrc, float32_t wo, float32_t wc, float32_t b, float32_t h, float32_t max_out);
extern float32_t ADRC_1_Calc(ADRC_1_t *adrc, float32_t w, float32_t ref);

typedef struct				//降一阶的二阶LADRC的参数（不变）结构体
{		
	float32_t Beita;        //离散化的LESO带宽β，β=e^-woh
	float32_t Beita_2;      //Beita_2=Beita*Beita
    float32_t L1;           //计算过程简化需要的参数
    float32_t L2	;   	//计算过程简化需要的参数
	float32_t ah;           //计算过程简化需要的参数
    float32_t parameter_1;  //计算过程简化需要的参数
    float32_t parameter_2;  //计算过程简化需要的参数
	float32_t wo;		   	//LESO带宽
	float32_t wc;		   	//控制器带宽
	float32_t b;            //控制器增益系数
	float32_t a;            //参数部分模型 a=1/Tm转矩常数（可通过系统辨识获得）
	float32_t h;            //步长（任务周期）
	float32_t max_out;      //最大输出限幅
} ADRC_2_parameter_t;

typedef struct				//降一阶的二阶LADRC的信号（随时间改变）结构体
{		
    float32_t ref;		 	//设定值
    float32_t angle;  		//云台角度
    float32_t w;  		 	//云台角速度
	float32_t u;      		//给系统的输入or控制器输出
	float32_t z[2];   		//LESO的状态变量的观测值 
	float32_t y[2];   		//LESO观测器输出
} ADRC_2_singal_t;

typedef struct				//降一阶的二阶LADRC
{		
	ADRC_2_parameter_t 	parameter;
	ADRC_2_singal_t   	singal;
} ADRC_2_t;

extern void ADRC_2_Init(ADRC_2_t *adrc, float32_t wo, float32_t wc, float32_t b, float32_t a, float32_t h, float32_t max_out);
extern float32_t ADRC_2_Calc(ADRC_2_t *adrc, float32_t angle, float32_t w, float32_t ref);

#endif

