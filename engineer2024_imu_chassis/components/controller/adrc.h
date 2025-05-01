#ifndef ADRC_H
#define ADRC_H
#include "struct_typedef.h"

typedef struct//一阶LADRC的参数（不变）结构体
{		
	fp32 Beita;   	//离散化的LESO带宽β，β=e^-woh
	fp32 Beita_2; 	//Beita_2=Beita*Beita
	fp32 wo;		//LESO带宽
	fp32 wc;		//控制器带宽
	fp32 b;       	//控制器增益系数	
	fp32 h;  		//步长（任务周期）
	fp32 max_out;	//最大输出限幅
} ADRC_1_parameter_t;

typedef struct		//一阶LADRC的信号（随时间改变）结构体
{		
    fp32 ref;		//设定值
    fp32 w;  		//云台角速度
	fp32 u;      	//给系统的输入or控制器输出
	fp32 z[2];   	//LESO的状态变量的观测值 
	fp32 y[2];   	//LESO观测器输出
} ADRC_1_singal_t;

typedef struct		//一阶LADRC
{			
	ADRC_1_parameter_t parameter;
	ADRC_1_singal_t singal;
} ADRC_1_t;

extern void ADRC_1_Init(ADRC_1_t *adrc,fp32 wo,fp32 wc,fp32 b,fp32 h,fp32 max_out);
extern fp32 ADRC_1_Calc(ADRC_1_t *adrc,fp32 w,fp32 ref);

typedef struct//降一阶的二阶LADRC的参数（不变）结构体
{		
	fp32 Beita;        //离散化的LESO带宽β，β=e^-woh
	fp32 Beita_2;      //Beita_2=Beita*Beita
    fp32 L1;           //计算过程简化需要的参数
    fp32 L2	;          //计算过程简化需要的参数
	fp32 ah;           //计算过程简化需要的参数
    fp32 parameter_1;  //计算过程简化需要的参数
    fp32 parameter_2;  //计算过程简化需要的参数
	fp32 wo;		   //LESO带宽
	fp32 wc;		   //控制器带宽
	fp32 b;            //控制器增益系数
	fp32 a;            //参数部分模型 a=1/Tm转矩常数（可通过系统辨识获得）
	fp32 h;            //步长（任务周期）
	fp32 max_out;      //最大输出限幅
} ADRC_2_parameter_t;

typedef struct//降一阶的二阶LADRC的信号（随时间改变）结构体
{		
    fp32 ref;		 	//设定值
    fp32 angle;  		//云台角度
    fp32 w;  		 	//云台角速度
	fp32 u;      		//给系统的输入or控制器输出
	fp32 z[2];   		//LESO的状态变量的观测值 
	fp32 y[2];   		//LESO观测器输出
} ADRC_2_singal_t;

typedef struct//降一阶的二阶LADRC
{		
	ADRC_2_parameter_t 	parameter;
	ADRC_2_singal_t   	singal;
} ADRC_2_t;

extern void ADRC_2_Init(ADRC_2_t *adrc,fp32 wo,fp32 wc,fp32 b,fp32 a,fp32 h,fp32 max_out);
extern fp32 ADRC_2_Calc(ADRC_2_t *adrc,fp32 angle,fp32 w,fp32 ref);

#endif
