#include "bsp_led.h"
#include "main.h"
extern TIM_HandleTypeDef htim5;
void led_argb(uint32_t argb)
{
    static uint8_t alpha;
    static uint16_t red, green, blue;
    alpha = (argb & 0xFF000000) >> 24;
    red = ((argb & 0x00FF0000) >> 16) * alpha;
    green = ((argb & 0x0000FF00) >> 8) * alpha;
    blue = ((argb & 0x000000FF) >> 0) * alpha;

    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);
}
