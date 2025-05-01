#include "detect_task.h"
#include "math.h"
#include "cmsis_os.h"

error_t error_list[GIMBAL_YAW_TOE + 1];
uint32_t system_time;

static void detect_init(void);

//检测任务
void detect_task(void const *pvParameters)
{
    system_time = xTaskGetTickCount();
    detect_init();
    vTaskDelay(DETECT_TASK_INIT_TIME);
    while(1)
    {
        system_time = xTaskGetTickCount();
        for (int i = 0; i < GIMBAL_YAW_TOE + 1; i++)
        {
            error_list[i].error_time = (system_time - error_list[i].new_time);
            //判断掉线
            if(error_list[i].error_time > error_list[i].set_offline_time)
            {
                error_list[i].error_exist = 1;
            }
            else
            {
                error_list[i].new_time = xTaskGetTickCount();
                error_list[i].error_exist = 0;
            }
        }
        vTaskDelay(DETECT_CONTROL_TIME);
    }
}

//记录时间
void detect_hook(uint8_t toe)
{
    error_list[toe].new_time = xTaskGetTickCount();
}

//得到错误列表
const error_t *get_error_list_point(void)
{
    return error_list;
}

static void detect_init(void)
{
    //设置离线时间、上线稳定时间、优先级
    uint16_t set_item[GIMBAL_YAW_TOE + 1] = {40,10,10,10,10};
    for (uint8_t i = 0; i < GIMBAL_YAW_TOE + 1; i++)
    {
        error_list[i].set_offline_time = set_item[i];
        error_list[i].error_exist = 1;
        error_list[i].new_time = xTaskGetTickCount();
    }
}
