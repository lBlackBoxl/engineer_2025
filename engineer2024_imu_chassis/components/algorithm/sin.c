#include "math.h"
#include "sin.h"
#define PI 3.141592653589793f
float sin_calc(float amplitude, float Ts)
{
	float sin_result = 0.0f;
	static uint32_t timeout = 0;
	static float frequency = 1.0f;
	timeout++;
	sin_result = amplitude * sin(2.0f * PI * frequency * (float)timeout * Ts);
	if(frequency * (float)timeout * Ts >= 1.0f)
	{	
		timeout = 0;
	}
	return sin_result;
}

