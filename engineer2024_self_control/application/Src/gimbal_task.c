#include "gimbal_task.h"
#include "MATH_LIB.h"
#include "main.h"
#include "cmsis_os.h"
#include "imu_task.h"


static const float32_t imu_filter[3] = {1.822751349748f, -0.8371810677374f,  0.01442971798952f};
static float32_t yaw_abs_angle_fliter[3] = {0.1142392673548f,   0.1189333870566f,   0.1142392673548f};
const float32_t *gimbal_INT_angle_point;
const float32_t *gimbal_INT_gyro_point;
const float32_t *gimbal_INT_accle_point;
float32_t yaw_absolute_angle,roll_absolute_angle,pitch_absolute_angle;
float32_t yaw_gyro;
void gimbal_task(void const *pvParameters)
{
	//等待陀螺仪任务更新陀螺仪数据
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
	
	  gimbal_INT_angle_point = get_imu_angle_point();
	  gimbal_INT_gyro_point = get_gyro_data_point();
	  gimbal_INT_accle_point = get_accel_data_point();
	
  //延时
		vTaskDelay(GIMBAL_TASK_INIT_TIME);
	while(1)
	{

		yaw_absolute_angle = (*(gimbal_INT_angle_point + IMU_YAW_ADDRESS_OFFSET));
		roll_absolute_angle = (*(gimbal_INT_angle_point + IMU_ROLL_ADDRESS_OFFSET));
		pitch_absolute_angle = (*(gimbal_INT_angle_point + IMU_PITCH_ADDRESS_OFFSET));
		yaw_gyro = *(gimbal_INT_gyro_point + IMU_GYRO_Y_ADDRESS_OFFSET);
		
		yaw_abs_angle_fliter[0] = yaw_abs_angle_fliter[1];
		yaw_abs_angle_fliter[1] = yaw_abs_angle_fliter[2];
		yaw_abs_angle_fliter[2] = yaw_abs_angle_fliter[1] * imu_filter[0] + yaw_abs_angle_fliter[0] * imu_filter[1] ;


		vTaskDelay(GIMBAL_CONTROL_TIME);
	}
}










