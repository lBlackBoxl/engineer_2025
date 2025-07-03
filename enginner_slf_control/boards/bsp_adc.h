#ifndef _BSP_ADC_H
#define _BSP_ADC_H

#include "struct_typedef.h"
#include "adc.h"

extern uint16_t adcx_get_chx_value(ADC_HandleTypeDef *ADCx, uint32_t ch);
extern void init_vrefint_reciprocal(void);
extern fp32 get_X_CtrStk_voltage(void);
extern fp32 get_Y_CtrStk_voltage(void);
extern uint16_t voltage_to_uint16_t(fp32 voltage);
extern void adc_init(void);

#endif 
