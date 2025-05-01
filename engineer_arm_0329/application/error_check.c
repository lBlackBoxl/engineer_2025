#include "error_check.h"
#include "cmsis_os.h"
#include "freertos.h"
#include "bsp_buzzer.h"

uint16_t error_info = 0;
uint16_t tick = 10;

uint16_t buzzer_psc = 500;
uint16_t buzzer_pwm = 15000;

void error_check(void const * argument)
{	
		//等待陀螺仪任务更新陀螺仪数据
		vTaskDelay(100);
			
		buzzer_off();
	
		while(1)
		{
				for(int i = 0; i < 5;i++)
			{
					if(fabs(arm.motor_DM_data[i].DM_motor_measure->motor_T) >= 19.0f)
					{
							error_info = 1;
					}
					else if(arm.motor_DM_data[i].DM_motor_measure->motor_err != 0)
					{
							error_info = 1;
					}
			}
			
			if(error_info == 1)
			{
					for(int i = 0; i < tick;i++)
					{
							buzzer_on(buzzer_psc, buzzer_pwm);
							HAL_Delay(50);
							buzzer_off();
							HAL_Delay(50);
					}
					error_info = 0;
					buzzer_off();
			}
			
				osDelay(1);
		}
}