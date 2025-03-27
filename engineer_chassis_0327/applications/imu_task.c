#include "main.h"
#include "imu_task.h"
#include "Freertos.h"
#include "cmsis_os.h"

void imu_task(void const *pvParameters)
{
		vTaskDelay(100);
	
		mpu_device_init();
		init_quaternion();

	  while (1)
			{
				mpu_get_data();
				imu_ahrs_update();
				imu_attitude_update(); 	
			}
			
}
