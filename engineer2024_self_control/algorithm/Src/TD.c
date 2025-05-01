#include "TD.h"
#include "arm_math.h"

float Sign(float x)
{
		if(x > 0.0f)
		{
				return 1.0f;
		}
		else if(x < 0.0f)
		{
				return -1.0f;
		}		
		else
		{
				return 0.0f;
		}
}

float fsg(float x,float d)
{
		return (Sign(x+d)-Sign(x-d))/2.0f;
}
//跟踪速度，滤波，任务频率，初始值
void TD_init(TD_t *td,float r,float N0,float h,float x)
{
		td->r  = r;
		td->h  = h;
		td->N0 = N0;
	 
	  td->ddx = 0.0f;
	  td->dx = 0.0f;	  
	  td->x = x;	
 	
}


//ADRC最速跟踪微分器TD，改进的算法fhan
float TD_calc(TD_t *td,float v)//安排ADRC过度过程
{
  float d=0,a0=0,y=0,a1=0,a2=0,a=0;
  float x1_delta=0;                            						//ADRC状态跟踪误差项
  x1_delta=td->x-v;                     									//用x1-v(k)替代x1得到离散更新公式
  td->h0=td->N0*td->h;       															//用h0替代h，解决最速跟踪微分器速度超调问题
  d=td->r*td->h0*td->h0;     															//d=rh^2;
  a0=td->h0*td->dx;                												//a0=h*x2
  y=x1_delta+a0;                               						//y=x1+a0
  a1=sqrt(d*(d+8*fabs(y)));                       				//a1=sqrt(d*(d+8*ABS(y))])
  a2=a0+Sign(y)*(a1-d)/2;                                 //a2=a0+sign(y)*(a1-d)/2;
  a=(a0+y)*fsg(y,d)+a2*(1-fsg(y,d));
  td->fhan = -td->r*(a/d)*fsg(a,d)-td->r*Sign(a)*(1-fsg(a,d)); 		//得到最速微分加速度跟踪量

	td->ddx = td->fhan;
  td->dx += td->h*td->ddx;									//跟新最速跟踪状态量微分dx  
	td->x  += td->h*td->dx;										//跟新最速跟踪状态量x
	
	return td->dx;
}

