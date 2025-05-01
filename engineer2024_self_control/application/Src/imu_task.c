#include "imu_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_spi.h"
#include "BMI088driver.h"
#include "AHRS.h"
#include "PID.h"

extern TIM_HandleTypeDef htim10;
extern SPI_HandleTypeDef hspi1;
static TaskHandle_t imu_task_local_handler;

#define BMI088_BOARD_INSTALL_SPIN_MATRIX	\
{0.0f, 1.0f, 0.0f},           				\
{-1.0f, 0.0f, 0.0f},                   		\
{0.0f, 0.0f, 1.0f}

static void imu_slove(float32_t gyro[3], float32_t accel[3], bmi088_real_data_t *bmi088);
static void gyro_cali(float32_t gyro[3]);
static void imu_temp_control(float32_t temp);
static void imu_cmd_spi_dma(void);
static void imu_temp_pwm(uint16_t pwm);

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF};

uint8_t cali_flag = CALI_ON;
uint32_t cali_time = 0;
float32_t cali_offset[3] = {0, 0, 0};
float32_t manual_offset[3] = {0.00165796024, -0.00154011324, -0.00131291943};
//0.00305729522, -0.00209091394, 0.000438075891
//0.00200114027f, -0.00341836875f, -0.00129196013f
volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;

bmi088_real_data_t bmi088_real_data;
float32_t gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
float32_t gyro_offset[3];
float32_t gyro_cali_offset[3];
float32_t accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
float32_t accel_offset[3];
float32_t accel_cali_offset[3];

static uint8_t first_temperate;
static pid_type_def imu_temp_pid;

static float32_t accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static float32_t accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static float32_t accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const float32_t fliter_num[3] = {0.2914341939862,   0.5834061966847,   0.2914341939862};

float32_t imu_gyro[3] = {0.0f, 0.0f, 0.0f};
static float32_t imu_accel[3] = {0.0f, 0.0f, 0.0f};
static float32_t imu_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float32_t imu_angle[3] = {0.0f, 0.0f, 0.0f};      

void imu_task(void const *pvParameters)
{
    osDelay(IMU_TASK_INIT_TIME);
    while(BMI088_init())
    {
        osDelay(100);
    }
	//SPI读取数据
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    //计算零漂
    imu_slove(imu_gyro, imu_accel, &bmi088_real_data);
	//控温PID初始化
    PID_init(&imu_temp_pid, PID_POSITION, TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
	//四元数初始化
	  AHRS_init(imu_quat);
    accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = imu_accel[0];
    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = imu_accel[1];
    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = imu_accel[2];
    //获取当前任务的任务句柄
    imu_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));
	//设置SPI通信频率
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
    imu_start_dma_flag = 1;
    while (1)
    {
        //等待SPI DMA传输
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }
        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
        }
        if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
        }
        if(accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }
		//计算角速度与加速度
        imu_slove(imu_gyro, imu_accel, &bmi088_real_data);
        //加速度计低通滤波
        accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];
        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + imu_accel[0] * fliter_num[2];
        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];
        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + imu_accel[1] * fliter_num[2];
        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];
        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + imu_accel[2] * fliter_num[2];
		AHRS_update(imu_quat, imu_gyro, accel_fliter_3);
		AHRS_get(imu_quat, imu_angle + IMU_YAW_ADDRESS_OFFSET, imu_angle + IMU_PITCH_ADDRESS_OFFSET, imu_angle + IMU_ROLL_ADDRESS_OFFSET);
		#if CALI_MODE
		gyro_cali(imu_gyro);
		#endif
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT1_ACCEL_Pin)
    {
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == INT1_GYRO_Pin)
    {
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == GPIO_PIN_0)
    {
        //唤醒任务
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(imu_task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}
static void imu_cmd_spi_dma(void)
{

        //开启陀螺仪的DMA传输
        if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
            gyro_update_flag |= (1 << IMU_SPI_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
            return;
        }
        //开启加速度计的DMA传输
        if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            accel_update_flag &= ~(1 << IMU_DR_SHFITS);
            accel_update_flag |= (1 << IMU_SPI_SHFITS);
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
            return;
        }
        if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
            accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
            return;
        }
}

void DMA2_Stream2_IRQHandler(void)
{
    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));
        //陀螺仪读取完毕
        if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);
            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
        }
        //加速度计读取完毕
        if(accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //温度读取完毕
        if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        imu_cmd_spi_dma();
        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}

//陀螺仪校准
static void gyro_cali(float32_t gyro[3])
{
	uint8_t i = 0;
	if(cali_flag == CALI_ON)
	{
		cali_time++;
		if(cali_time >= START_CALI_TIME)
		{
			cali_offset[0] += gyro[0];
			cali_offset[1] += gyro[1];
			cali_offset[2] += gyro[2];
		}
		if(cali_time >= MAX_CALI_TIME)
		{
			for(i = 0; i < 3; i++)
				cali_offset[i] /= (float)(MAX_CALI_TIME - START_CALI_TIME);
			cali_flag = CALI_FINISH;
		}
	}
}

//计算角速度与加速度
static void imu_slove(float32_t gyro[3], float32_t accel[3], bmi088_real_data_t *bmi088)
{
	uint8_t i = 0;
    for (i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];	
	}
	#if CALI_MODE
	if(cali_flag == CALI_FINISH) 
	{
		for(i = 0; i< 3; i++)
			gyro[i] = gyro[i] - cali_offset[i];
	}
	#else
	for(i = 0; i< 3; i++)
			gyro[i] = gyro[i] - manual_offset[i];
	#endif
}
//BMI088控温
static void imu_temp_control(float32_t temp)
{
    uint16_t temp_pwm;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        PID_calc(&imu_temp_pid, temp, 40.0f);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        temp_pwm = (uint16_t)imu_temp_pid.out;
        imu_temp_pwm(temp_pwm);
    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        if (temp > 40.0f)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                first_temperate = 1;
                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }
        imu_temp_pwm(MPU6500_TEMP_PWM_MAX - 1);
    }
}

static void imu_temp_pwm(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
}

//获取四元数
const float32_t *get_imu_quat_point(void)
{
    return imu_quat;
}
//获取欧拉角
const float32_t *get_imu_angle_point(void)
{
    return imu_angle;
}
//角速度指针
extern const float32_t *get_gyro_data_point(void)
{
    return imu_gyro;
}
//加速度计指针
extern const float32_t *get_accel_data_point(void)
{
    return imu_accel;
}
