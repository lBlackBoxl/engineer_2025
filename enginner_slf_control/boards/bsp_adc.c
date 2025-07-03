#include "main.h"
#include "adc.h"
#include "bsp_adc.h"
//ADC_CHANNEL_8 ADC_CHANNEL_9
volatile fp32 voltage_vrefint_proportion=8.0586080586080586080586080586081e-4f;

//得到相应adc相应通道的值
uint16_t adcx_get_chx_value(ADC_HandleTypeDef *ADCx, uint32_t ch)
{
	   static ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ch;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;//ADC_SAMPLETIME_3CYCLES;

    if (HAL_ADC_ConfigChannel(ADCx, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
	
    HAL_ADC_Start(ADCx);

    HAL_ADC_PollForConversion(ADCx, 10);
    return (uint16_t)HAL_ADC_GetValue(ADCx);

}
//内部VREFINT电压的使用(得到调整系数)
void init_vrefint_reciprocal(void)
{
    uint8_t i = 0;
    uint32_t total_adc = 0;
    for(i = 0; i < 200; i++)
    {
        total_adc += adcx_get_chx_value(&hadc1, ADC_CHANNEL_VREFINT);
    }

    voltage_vrefint_proportion = 200 * 1.2f / total_adc;
}
//获得遥感X电压
fp32 get_X_CtrStk_voltage(void)
{
    fp32 voltage;
    uint16_t adcx = 0;

    adcx = adcx_get_chx_value(&hadc1, ADC_CHANNEL_8);
    voltage =  (fp32)adcx * voltage_vrefint_proportion;

    return voltage;
}
//获得遥感Y电压
fp32 get_Y_CtrStk_voltage(void)
{
    fp32 voltage;
    uint16_t adcx = 0;

    adcx = adcx_get_chx_value(&hadc1, ADC_CHANNEL_9);
    voltage =  (fp32)adcx * voltage_vrefint_proportion;

    return voltage;
}
//处理电压数据（映射到0-1000）
uint16_t voltage_to_uint16_t(fp32 voltage)
{
	return (uint16_t)(voltage*1000/3.3);
}

void adc_init()
{
	  init_vrefint_reciprocal();
}

