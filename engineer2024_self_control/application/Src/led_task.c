#include "led_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "bsp_led.h"

uint32_t color[RGB_COLOR + 1] = {0xFF0000FF, 0x0000FF00, 0xFFFF0000, 0x000000FF, 0xFF00FF00, 0x00FF0000, 0xFF0000FF};

void led_task(void const * argument)
{
    uint16_t i, j;
    float32_t delta_alpha, delta_red, delta_green, delta_blue;
    float32_t alpha, red, green, blue;
    uint32_t argb;
    while(1)
    {
        for(i = 0; i < RGB_COLOR; i++)
        {
            alpha = (color[i] & 0xFF000000) >> 24;
            red = ((color[i] & 0x00FF0000) >> 16);
            green = ((color[i] & 0x0000FF00) >> 8);
            blue = ((color[i] & 0x000000FF) >> 0);
            delta_alpha = (float32_t)((color[i + 1] & 0xFF000000) >> 24) - (float32_t)((color[i] & 0xFF000000) >> 24);
            delta_red = (float32_t)((color[i + 1] & 0x00FF0000) >> 16) - (float32_t)((color[i] & 0x00FF0000) >> 16);
            delta_green = (float32_t)((color[i + 1] & 0x0000FF00) >> 8) - (float32_t)((color[i] & 0x0000FF00) >> 8);
            delta_blue = (float32_t)((color[i + 1] & 0x000000FF) >> 0) - (float32_t)((color[i] & 0x000000FF) >> 0);
            delta_alpha /= RGB_PERIOD;
            delta_red /= RGB_PERIOD;
            delta_green /= RGB_PERIOD;
            delta_blue /= RGB_PERIOD;
            for(j = 0; j < RGB_PERIOD; j++)
            {
                alpha += delta_alpha;
                red += delta_red;
                green += delta_green;
                blue += delta_blue;
                argb = ((uint32_t)(alpha)) << 24 | ((uint32_t)(red)) << 16 | ((uint32_t)(green)) << 8 | ((uint32_t)(blue)) << 0;
                led_argb(argb);
                osDelay(1);
            }
        }
    }
}
