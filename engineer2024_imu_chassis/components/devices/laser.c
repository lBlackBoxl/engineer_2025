#include "laser.h"
#include "tim.h"
extern TIM_HandleTypeDef htim3;
//¿ªÆô¼¤¹â
void laser_on(void)
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 8399);
}
void laser_off(void)
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
}
