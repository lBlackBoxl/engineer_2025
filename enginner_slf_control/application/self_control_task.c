#include "main.h"
#include "self_control_task.h"
#include "cmsis_os.h"
#include "STS_app.h"
#include "bsp_adc.h"
#include "usart.h"
#include "Rocker_app.h"

fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        fp32 len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        fp32 len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

#define PI 3.1415926
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

extern UART_HandleTypeDef huart7;
float Present_Position[6];

void self_control_init(void);
extern int ReadPos(int ID);//¶ÁÎ»ÖÃ
extern int writeWord(uint8_t ID, uint8_t MemAddr, uint16_t wDat);//Ð´2¸ö×Ö½Ú

fp32 position_allowance[4] = {1.49869919, -2.8854177, -0.829883575, -0.420310736};

void self_control_task(void const *argument)
{
    vTaskDelay(200);
		
		self_control_init();
	
    while (1)
    {
				for(int i = 0; i < 4; i++)
				{						
						Present_Position[i] = -rad_format(((ReadPos(i+1) - 2048) * 2 * PI / 4096) - position_allowance[i]);
				}
				rocker_scan(Present_Position + 4);
				osDelay(1);
    }
}

void self_control_init()
{
		STS_Init();
		adc_init();
}