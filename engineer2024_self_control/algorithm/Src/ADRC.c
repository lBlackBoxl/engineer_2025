#include "ADRC.h"
#include "math.h"
#include "MATH_LIB.h"
#include "main.h"

//一般用于一阶的速度控制，或两个一阶ADRC串级进行角度控制使用
//一阶adrc参数初始化
void ADRC_1_Init(ADRC_1_t *adrc,float32_t wo,float32_t wc,float32_t b,float32_t h,float32_t max_out)
{
    if (adrc == NULL)
    {
        return;
    }	
		adrc->parameter.wo = wo;
		adrc->parameter.wc = wc;
		adrc->parameter.b	 = b;
		adrc->parameter.h	 = h;	
		adrc->parameter.max_out = max_out;	
}

//全阶的LESO，阶数为2-0=2
void ADRC_1_LESO(ADRC_1_t *adrc)
{
		//z[0]=h*z2 - y*(2*BB - 2) + z1*(2*BB - 1) + bo*h*u
		adrc->singal.z[0] = adrc->singal.z[0]*(adrc->parameter.Beita*2-1)+adrc->parameter.h*adrc->singal.z[1]
							-adrc->singal.w*(adrc->parameter.Beita*2-2)+adrc->parameter.b*adrc->parameter.h*adrc->singal.u;
		//z[1]=z2 + (y*(BB^2 - 2*BB + 1))/h - (z1*(BB^2 - 2*BB + 1))/h
		adrc->singal.z[1] = -(adrc->parameter.Beita_2-adrc->parameter.Beita*2+1)*adrc->singal.z[0]/adrc->parameter.h
							+adrc->singal.z[1]+(adrc->parameter.Beita_2-adrc->parameter.Beita*2+1)*adrc->singal.w/adrc->parameter.h;

		//yd[0]= BB^2*z1 - y*(BB^2 - 1)
		adrc->singal.y[0] = adrc->parameter.Beita_2*adrc->singal.z[0]-adrc->singal.w*(adrc->parameter.Beita_2-1);
		//yd[1]=z2 + (y*(BB^2 - 2*BB + 1))/h - (z1*(BB^2 - 2*BB + 1))/h
		adrc->singal.y[1] = -(adrc->parameter.Beita_2-adrc->parameter.Beita*2+1)*adrc->singal.z[0]/adrc->parameter.h
							+adrc->singal.z[1]	+(adrc->parameter.Beita_2-adrc->parameter.Beita*2+1)*adrc->singal.w/adrc->parameter.h;	
}

//一阶adrc计算
float32_t ADRC_1_Calc(ADRC_1_t *adrc,float32_t w,float32_t ref)
{
    if (adrc == NULL)
    {
        return 0.0f;
    }
		//更新参数
		adrc->parameter.Beita   = exp2f(-adrc->parameter.wo*adrc->parameter.h);
		adrc->parameter.Beita_2 = adrc->parameter.Beita*adrc->parameter.Beita;
		
		float32_t kp;
		kp = adrc->parameter.wc;
		adrc->singal.ref = ref;		
		adrc->singal.w = w;
		//扩张观测器
		ADRC_1_LESO(adrc);
		//线性控制器u0=kp*(ref-y)-kd*dy/dt-yd;
		adrc->singal.u = kp * (adrc->singal.ref-adrc->singal.y[0]) - adrc->singal.y[1] ;
		//u = u0/b
		adrc->singal.u = adrc->singal.u/adrc->parameter.b;
		//对u限幅
		float32_limit(adrc->singal.u, -adrc->parameter.max_out, adrc->parameter.max_out);
		
		return adrc->singal.u;
}

//一般用于二阶的角度控制
//adrc参数初始化
void ADRC_2_Init(ADRC_2_t *adrc,float32_t wo,float32_t wc,float32_t b,float32_t a,float32_t h,float32_t max_out)
{
    if (adrc == NULL)
    {
        return;
    }	
	adrc->parameter.wo = wo;
	adrc->parameter.wc = wc;
	adrc->parameter.b	 = b;
	adrc->parameter.a	 = a;
	adrc->parameter.h	 = h;	
	adrc->parameter.max_out = max_out;			
}

//降一阶的LESO，阶数为3-1=2
void ADRC_2_LESO(ADRC_2_t *adrc)
{

	

	    adrc->singal.z[0] =  adrc->singal.z[0]*(1-adrc->parameter.L2*adrc->parameter.parameter_1-adrc->parameter.L1)
												+adrc->singal.z[1]*adrc->parameter.parameter_1
												+adrc->singal.u*adrc->parameter.b*adrc->parameter.parameter_1
												+adrc->singal.w*(adrc->parameter.L1+adrc->parameter.L2*adrc->parameter.parameter_1);
	
		adrc->singal.z[1] = -adrc->singal.z[0]*adrc->parameter.L2*adrc->parameter.parameter_2
												+adrc->singal.z[1]+adrc->parameter.parameter_2
												-adrc->singal.u*adrc->parameter.a*adrc->parameter.b*adrc->parameter.parameter_1
												+adrc->singal.w*adrc->parameter.L2*adrc->parameter.parameter_2;
		adrc->singal.y[0] =  adrc->singal.z[0]*(1-adrc->parameter.L1)+adrc->singal.w*adrc->parameter.L1;

		adrc->singal.y[1] = -adrc->singal.z[0]*adrc->parameter.L2 + adrc->singal.z[1]+adrc->singal.w*adrc->parameter.L2;

}

//二阶adrc计算
float32_t ADRC_2_Calc(ADRC_2_t *adrc,float32_t angle,float32_t w,float32_t ref)
{
    if (adrc == NULL)
    {
        return 0.0f;
    }
		//更新计算过程简化需要的参数，计算量较大
		adrc->parameter.Beita   = exp2f(-adrc->parameter.wo*adrc->parameter.h);
		adrc->parameter.Beita_2 = adrc->parameter.Beita*adrc->parameter.Beita;
		adrc->parameter.ah      = adrc->parameter.a*adrc->parameter.h;
		adrc->parameter.L1      = (adrc->parameter.ah * adrc->parameter.ah - 2 * adrc->parameter.ah - 2 * adrc->parameter.Beita_2 + 2)
												      /(adrc->parameter.ah * adrc->parameter.ah - 2 * adrc->parameter.ah + 2);
		
		adrc->parameter.L2      =  (adrc->parameter.ah * adrc->parameter.ah - 2 * adrc->parameter.ah - 2 * adrc->parameter.Beita + 2)
												      *(adrc->parameter.ah * adrc->parameter.ah - 2 * adrc->parameter.ah - 2 * adrc->parameter.Beita + 2)
													    /(- adrc->parameter.ah * adrc->parameter.ah * adrc->parameter.ah * adrc->parameter.h
													    +4 * adrc->parameter.ah * adrc->parameter.ah * adrc->parameter.h - 6 * adrc->parameter.ah * adrc->parameter.h
													    +4 * adrc->parameter.h);
   
		adrc->parameter.parameter_1 = adrc->parameter.h - adrc->parameter.ah * adrc->parameter.h / 2;
   	adrc->parameter.parameter_2 = adrc->parameter.ah * adrc->parameter.ah / 2 - adrc->parameter.ah + 1;
		
		//控制器增益kp,kd
		float32_t kp,kd;
		kd = 2*adrc->parameter.wc;
		kp = adrc->parameter.wc*adrc->parameter.wc;
		adrc->singal.ref   = ref;		
		adrc->singal.w  	 = w;
		adrc->singal.angle = angle;
		//扩张观测器
		ADRC_2_LESO(adrc);
		//线性控制器u0=kp*(ref-y)-kd*dy/dt-yd;
		adrc->singal.u = kp * rad_format(adrc->singal.ref-adrc->singal.angle) - kd * adrc->singal.y[0] - adrc->singal.y[1];
		//u = u0/b
		adrc->singal.u = adrc->singal.u/adrc->parameter.b;
		//对u限幅
		float32_limit(adrc->singal.u, -adrc->parameter.max_out, adrc->parameter.max_out);
		
		return adrc->singal.u;
}

