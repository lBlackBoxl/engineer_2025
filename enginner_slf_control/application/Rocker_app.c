#include "main.h"
#include "adc.h"
#include "Rocker_app.h"
#include "struct_typedef.h"

void rocker_scan(fp32* value)
{
		int temp = 0;
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 50);
		for(int i = 0; i < 2; i++)
		{
				HAL_ADC_Start(&hadc1);
				HAL_ADC_PollForConversion(&hadc1, 50);
				temp = HAL_ADC_GetValue(&hadc1);
				*(value + i) = -((temp * 2000 / 4096) - 1000);
		}
		
		HAL_ADC_Stop(&hadc1);
}