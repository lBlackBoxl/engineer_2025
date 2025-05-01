#include "Butterworth_Filter.h"



/**
 * @param[in]       Filter        	Butter_worth_Filter_t结构体
 * @param[in]       Fc          	截止频率
 * @param[in]       Ts        		任务周期
 */

void Butter_worth_Filter_init(Butter_worth_Filter_t *Filter,fp32 Fc,fp32 Ts)
{
		Filter->Fc = Fc;
		Filter->Ts = Ts;
		Filter->Yout[0] = 0;
		Filter->Yout[1] = 0;
		Filter->Yout[2] = 0;
		Filter->Xin[0] =  0;
		Filter->Xin[1] =  0;
		Filter->Xin[2] =  0;
		
		//Wc = 2 * pi * Fc，Fc为截止频率	
		Filter->Wc = 2.0f * 3.1415926f *Fc;
		//Dp为阻尼系数（damp的缩写），一般设置为0.707
		Filter->Dp = 0.707f;
		//计算b0，a0，a1，a2，为了方便演示放在函数里了，实际上应该把这四个数或者原始的Wc，Ts，Dp算好了做成宏定义，减少计算量
		Filter->b0 = Filter->Wc * Filter->Wc *Filter->Ts * Filter->Ts;
		Filter->a0 = 4.0f + 4.0f * Filter->Dp * Filter->Wc * Ts + Filter->Wc * Filter->Wc * Filter->Ts * Filter->Ts;
		Filter->a1 = -8.0f +2.0f * Filter->Wc * Filter->Wc * Ts * Filter->Ts;
		Filter->a2 = 4.0f - 4.0f * Filter->Dp * Filter->Wc * Ts + Filter->Wc * Filter->Wc * Filter->Ts * Filter->Ts;

}


/**
 * @param[in]       Filter        	Butter_worth_Filter_t结构体
 * @param[in]       xin          	输入
 */

fp32 Butter_worth_Filter_calc(Butter_worth_Filter_t *Filter,fp32 xin)
{
	
    Filter->Xin[2] = xin;
    //唯一的必须要做的计算，如果你觉得计算量还是很大，可以开启FPU减轻运算负担
    Filter->Yout[2] = (Filter->b0 * Filter->Xin[2] + 2.0f * Filter->b0 * Filter->Xin[1] + Filter->b0 * Filter->Xin[0] - Filter->a1 * Filter->Yout[1] - Filter->a2 * Filter->Yout[0]) / Filter->a0;
    //Vo(n-2) = Vo(n-1)，这是因为经过了一个任务周期后，第n-1个输出信号变成了第n-2个输出信号，同理第n个输出信号变成了第n-1个输出信号
    Filter->Yout[0] = Filter->Yout[1];
    //Vo(n-1) = Vo(n)
    Filter->Yout[1] = Filter->Yout[2];
    //Vi(n-2) = Vi(n-1)
    Filter->Xin[0] = Filter->Xin[1];
    //Vi(n-1) = Vi(n)
    Filter->Xin[1] = Filter->Xin[2];
    //最后输出的是Y(n)
	
    return Filter->Yout[2];
}
