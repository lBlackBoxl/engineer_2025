#include "can_solve.h"
#include "main.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "shoot_task.h"
#include "gimbal_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
#define get_motor_measure(ptr, data)                               	\
{                                                                   \
	(ptr)->last_ecd = (ptr)->ecd;                                   \
	(ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
	(ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
	(ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
	(ptr)->temperate = (data)[6];                                   \
}
//电机接收结构体
struct Tx_exCanIdInfo txCanIdEx;
union Tx_exId_uint32 trans_tx;
static uint8_t cybergear_runmode;
static uint16_t cybergear_index;
static float32_t cybergear_param_ref;
struct Rx_exCanIdInfo rxCanIdEx;
union Rx_exId_uint32 trans_rx;

motor_encoder_measure_t motor_encoder;

motor_measure_t motor_gimbal[8];

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
int8_t fire_flag;
float32_t bullet_speed;
uint8_t color_flag;
uint8_t enemy_min_flag;
static void get_referee_measure(uint8_t rx_data[12]);
static void get_wz_set(uint8_t rx_data[12]);
static void get_chassis_absolute_angle(uint8_t rx_data[8]);
static void get_hit_data(uint8_t rx_data[8]);
static void get_cybergear_motor_measure(struct Rx_exCanIdInfo exid, uint8_t rx_data[8]);
static void get_encoder_measure(motor_encoder_measure_t* motor_encoder , uint8_t rx_data[8]);
//联合体解算数据
union FloatUint8_t t;
uint8_t i;
//自定义控制器数据
uint8_t self_control_flag;
uint8_t position_flag;
uint8_t FIRE_FLAG;
uint8_t STOP_FLAG;
//CAN中断回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    static uint8_t i = 0;
    uint8_t rx_data[8];
	CAN_RxHeaderTypeDef rx_header;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	trans_rx.aa = rx_header.ExtId;
	rxCanIdEx = trans_rx.exid;
    if (hcan == &hcan1)
    {
		//标准帧
//			get_encoder_measure(&motor_encoder,rx_data );
      switch (rx_header.StdId)
      {
			case ENCODER_MASTER_ID:
			     {
			       get_encoder_measure(&motor_encoder,rx_data );
				     break;
			     }
				
			case CAN_FRIC1_MOTOR_ID:   //0x201    [0]
			case CAN_FRIC2_MOTOR_ID:   //0x202    [1]
            {
                i = rx_header.StdId - CAN_FRIC1_MOTOR_ID;
				get_motor_measure(&motor_gimbal[i], rx_data);
                if(i < 3)
				{
					detect_hook(GIMBAL_FRIC1_TOE + i);
				}
				else
					detect_hook(GIMBAL_FRIC1_TOE + i - 1);
				break;
            }
			case CAN_PIT_MOTOR_ID:	   //0x205    [7]
			{
				get_motor_measure(&motor_gimbal[7], rx_data);
				break;
			}
			case CAN_YAW_MOTOR_ID:         //0x20A   [6]
			{
				get_motor_measure(&motor_gimbal[6], rx_data);
				break;
			}
			case CAN_TRIGGER1_MOTOR_ID:    //0x206  
            {
                i = rx_header.StdId - CAN_FRIC1_MOTOR_ID;
				get_motor_measure(&motor_gimbal[i], rx_data);
				detect_hook(GIMBAL_FRIC1_TOE + i - 1);
				break;
            }
			
            default:
			{
				break;
			}
        }
		//扩展帧
		switch(rxCanIdEx.motor_CanId)
		{
			case 127:
			{
			get_cybergear_motor_measure(rxCanIdEx, rx_data);
				break;
			}

			default:
				break;
		}

    }
    if(hcan == &hcan2)
    {
        switch (rx_header.StdId)
        {
//			case CAN_TRIGGER1_MOTOR_ID:    //0x206  
//            {
//                i = rx_header.StdId - CAN_FRIC1_MOTOR_ID;
//				get_motor_measure(&motor_gimbal[i], rx_data);
//				detect_hook(GIMBAL_FRIC1_TOE + i - 1);
//				break;
//            }
            case CAN_REFEREE_ID:
			{
				get_referee_measure(rx_data);
				break;
			}
			case CAN_YAW_MOTOR_ID:         //0x207   [3]
			{
				get_motor_measure(&motor_gimbal[6], rx_data);
				break;
			}
			case CAN_ANGLE_ID:
			{
				get_chassis_absolute_angle(rx_data);
				break;
			}
			case CAN_HIT_ID:
			{
				get_hit_data(rx_data);
				break;
			}
			default:
			{
				break;
			}
        }
    }
}

//通信类型7，设置电机ID
//void Can_crbergear_canID_set()
//通信类型18，设置单个参数
void Can_crbergear_runmode_set(uint8_t id, uint16_t master_id, uint8_t runmode)
{
	cybergear_index = 0x7005;
	cybergear_runmode = runmode;
	uint32_t send_mail_box;
	txCanIdEx.id = id;
	txCanIdEx.data = master_id;
	txCanIdEx.mode = 18;
	txCanIdEx.res = 0;
	trans_tx.exid = txCanIdEx;
	gimbal_tx_message.ExtId = trans_tx.aa;
	gimbal_tx_message.IDE = CAN_ID_EXT;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;
	for(uint8_t i=0;i<8;i++)
	{
		gimbal_can_send_data[i] = 0;
	}
	memcpy(&gimbal_can_send_data[0], &cybergear_index, 2);
	memcpy(&gimbal_can_send_data[4], &cybergear_runmode, 1);
	HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}
//通信类型3，使能电机
void Can_cybergear_enable(uint8_t id, uint16_t master_id)
{
	uint32_t send_mail_box;
	txCanIdEx.id = id;
	txCanIdEx.data = master_id;
	txCanIdEx.mode = 3;
	txCanIdEx.res = 0;
	txCanIdEx.data = 0; 
	trans_tx.exid = txCanIdEx;
	gimbal_tx_message.ExtId = trans_tx.aa;
	gimbal_tx_message.IDE = CAN_ID_EXT;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;

	HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);

}
//控制参数写入iq_ref, loc_ref, spd_ref
void Can_cybergear_param_write(uint8_t id, uint16_t master_id, uint16_t index, float32_t param_ref)
{
	cybergear_index = index;
	cybergear_param_ref = param_ref;
	uint32_t send_mail_box;
	txCanIdEx.id = id;
	txCanIdEx.data = master_id;
	txCanIdEx.mode = 18;
	txCanIdEx.res = 0;
	
	trans_tx.exid = txCanIdEx;
	gimbal_tx_message.ExtId = trans_tx.aa;
	gimbal_tx_message.IDE = CAN_ID_EXT;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;
	
	for(uint8_t i=0;i<8;i++)
	{
		gimbal_can_send_data[i] = 0;
	}
	memcpy(&gimbal_can_send_data[0], &cybergear_index, 2);
	memcpy(&gimbal_can_send_data[4], &cybergear_param_ref, 4);
	
	HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}
//电机停止运行
void Can_cybergear_stop(uint8_t id,uint16_t master_id)
{
	uint32_t send_mail_box;
	txCanIdEx.id = id;
	txCanIdEx.data = master_id;
	txCanIdEx.mode = 4;
	txCanIdEx.res = 0;
	trans_tx.exid = txCanIdEx;
	gimbal_tx_message.ExtId = trans_tx.aa;
	gimbal_tx_message.IDE = CAN_ID_EXT;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;
	for(uint8_t i=0;i<8;i++)
	{
		gimbal_can_send_data[i] = 0;
	}
	HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);	
}
//中空编码器控制电机停转
void Can_encoder_stop()
{
  uint32_t send_mail_box;
	gimbal_tx_message.StdId = 0x001;
	gimbal_tx_message.IDE = CAN_ID_STD ;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;
	for(uint8_t i = 0; i < 7; i++)
	{
		gimbal_can_send_data[i] = 0xFF;
	}
	gimbal_can_send_data[7] = 0xFD;
  HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);	
}

void Can_encoder_start()//使能电机
{
  uint32_t send_mail_box;
	gimbal_tx_message.StdId = 0x001;//电机ID
	gimbal_tx_message.IDE = CAN_ID_STD ;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;
	for(uint8_t i = 0; i < 7; i++)
	{
		gimbal_can_send_data[i] = 0xFF;
	}
	gimbal_can_send_data[7] = 0xFC;
  HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);	
}

void Can_encoder_set()//设置为力矩模式
{
  uint32_t send_mail_box;
	gimbal_tx_message.StdId = 0x001;
	gimbal_tx_message.IDE = CAN_ID_STD ;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;
	for(uint8_t i = 0; i < 7; i++)
	{
		gimbal_can_send_data[i] = 0xFF;
	}
	gimbal_can_send_data[7] = 0xF9;
  HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);	
}

void Can_encoder_current_mode(int16_t current)//力矩模式发送电流
{
  uint32_t send_mail_box;
	int16_t temp;
	gimbal_tx_message.StdId = 0x001;
	gimbal_tx_message.IDE = CAN_ID_STD;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;
	
 gimbal_can_send_data[0] = 0x8000>>8;
 gimbal_can_send_data[1] = 0x8000&0xFF;//
 gimbal_can_send_data[2] = 0x900>>4;;
 gimbal_can_send_data[3] = ((0x900&0xF)<<4) + (0x800>>8);
 gimbal_can_send_data[4] = 0x800 &0xFF;
 gimbal_can_send_data[5] = 0x800>>4;
 gimbal_can_send_data[6] = ((0x800 &0xF)<<4) + (0x900 >>8);
 gimbal_can_send_data[7] = 0x900&0xFF;;
  HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);	
}
void Can_encoder_positoin_mode()//力矩模式发送电流
{
  uint32_t send_mail_box;
	int16_t temp;
	gimbal_tx_message.StdId = 0x001;
	gimbal_tx_message.IDE = CAN_ID_STD;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;

 gimbal_can_send_data[0] = 0xFFFF>>8;
 gimbal_can_send_data[1] = 0xFFFF&0xFF;//
 gimbal_can_send_data[2] = 0x900>>4;
 gimbal_can_send_data[3] = ((0x900&0xF)<<4) + (0x800>>8);
 gimbal_can_send_data[4] = 0x800 &0xFF;
 gimbal_can_send_data[5] = 0x800>>4;
 gimbal_can_send_data[6] = ((0x800 &0xF)<<4) + (0xFFF >>8);
 gimbal_can_send_data[7] = 0xFFF&0xFF;

  HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);	
}














//发送yaw电机的电流
void CAN_cmd_yaw(int16_t current1)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_YAW_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (current1 >> 8);
    gimbal_can_send_data[1] = current1;
    gimbal_can_send_data[2] = 0;
    gimbal_can_send_data[3] = 0;
    gimbal_can_send_data[4] = 0;
    gimbal_can_send_data[5] = 0;
    gimbal_can_send_data[6] = 0;
    gimbal_can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}
//向底盘发送开火、角度信息、角速度
union FloatUint8_t shift;
void CAN_cmd_state(float32_t angle)
{
	shift.a = angle;
	uint8_t i;
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_STATE_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = auto_aim_receive_msg.rx_data.fire;
	for(i = 0; i<4;i++) gimbal_can_send_data[i+1] = shift.b[i];
    gimbal_can_send_data[5] = 0;
    gimbal_can_send_data[6] = 0;
    gimbal_can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
	
}
//向底盘发送角速度
void CAN_send_gyro(float32_t gimbal_gyro)
{
	shift.a = gimbal_gyro;
	uint8_t i;
	uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_SEND_GYRO_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    for(i = 0; i<4;i++) gimbal_can_send_data[i] = shift.b[i];
	gimbal_can_send_data[4] = 0;
	gimbal_can_send_data[5] = 0;
    gimbal_can_send_data[6] = 0;
    gimbal_can_send_data[7] = 0;
	HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

union FloatUint8_t Vw;

void CAN_send_Speed(vel_rx_data_t *command)
{   Vw.a = command->vw;
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_SEND_SPEED_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
	  gimbal_can_send_data[0] = (command->vx >> 8);
    gimbal_can_send_data[1] = command->vx;
    gimbal_can_send_data[2] = (command->vy >> 8);
    gimbal_can_send_data[3] = command->vy;
    gimbal_can_send_data[4] = Vw.b[0];
    gimbal_can_send_data[5] = Vw.b[1];
    gimbal_can_send_data[6] = Vw.b[2];
    gimbal_can_send_data[7] = Vw.b[3];
    HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

//发送pitch电机的电流
void CAN_cmd_pitch(int16_t current1,int16_t trigger_current)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_PITCH_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (current1 >> 8);
    gimbal_can_send_data[1] = current1;
    gimbal_can_send_data[2] = (trigger_current>>8);
    gimbal_can_send_data[3] = trigger_current;
    gimbal_can_send_data[4] = 0;
    gimbal_can_send_data[5] = 0;
    gimbal_can_send_data[6] = 0;
    gimbal_can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}
//发送摩擦轮的电流
void CAN_cmd_shoot(int16_t current1,int16_t current2, int16_t current3, int16_t current4)
{   
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_SHOOT_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (current1 >> 8);
    gimbal_can_send_data[1] = current1;
    gimbal_can_send_data[2] = (current2 >> 8);
    gimbal_can_send_data[3] = current2;
    gimbal_can_send_data[4] = 0;//(current3 >> 8);
    gimbal_can_send_data[5] = 0;//current3;
    gimbal_can_send_data[6] = 0;//(current4 >> 8);
    gimbal_can_send_data[7] = 0;//current4;
    HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

//发送拨弹盘电流
void CAN_cmd_trigger(int16_t current1, int16_t current2)
{
	uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_TRIGGER_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = 0;
    gimbal_can_send_data[1] = 0;
    gimbal_can_send_data[2] = (current1 >> 8);
    gimbal_can_send_data[3] = current1;
    gimbal_can_send_data[4] = 0;
    gimbal_can_send_data[5] = 0;
    gimbal_can_send_data[6] = 0;
    gimbal_can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}


//通过底盘获取裁判系统的相关数据
static void get_referee_measure(uint8_t rx_data[8])
{
	uint8_t i;
	for(i=0;i<4;i++)
		t.b[i] = rx_data[i];
	shoot_control.bullet_speed = t.a;
	color_flag = rx_data[4];
	shoot_control.heat_1 = rx_data[5] << 8 | rx_data[6];
}

//获得击打目标数据
static void get_hit_data(uint8_t rx_data[8])
{
	uint8_t i;
	for(i=0;i<8;i++)
	   auto_aim_send_msg.tx_data.hit[i] = 1;
	auto_aim_send_msg.tx_data.hit[1] = rx_data[2] ;
	auto_aim_send_msg.tx_data.hit[5] = rx_data[0];
	auto_aim_send_msg.tx_data.hit[6] = rx_data[1];
	auto_aim_send_msg.tx_data.hit[7] = 0; //基地默认不打
}

//获取底盘绝对角度
static void get_chassis_absolute_angle(uint8_t rx_data[8])
{
	for(i = 0;i<4;i++) 
		t.b[i] = rx_data[i];
	chassis_absolute_angle = t.a;
	auto_aim_send_msg.tx_data.is_balance[0] = rx_data[4];
	auto_aim_send_msg.tx_data.is_balance[1] = rx_data[5];
	auto_aim_send_msg.tx_data.is_balance[2] = rx_data[6];
}

//获得cybergear数据
static void get_cybergear_motor_measure(struct Rx_exCanIdInfo exid, uint8_t rx_data[8])
{
	if(exid.communication_mode == 2 && exid.id == 0 && exid.calibration_err == 0
		&& exid.hall_coding_err == 0 && exid.hightemperature_err == 0
	    && exid.magnetic_coding_err == 0 && exid.overcurrent_err == 0
	    && exid.overcurrent_err == 0 && exid.undervoltage_err == 0)
	{
		cybergear_measure.mode = exid.mode;
		cybergear_measure.angle_i = (rx_data[0] << 8 | rx_data[1])^0x8000;
		cybergear_measure.gyro_i = (rx_data[2] << 8 | rx_data[3])^0x8000;
		cybergear_measure.torque_i = (rx_data[4] << 8 | rx_data[5])^0x8000;
		cybergear_measure.temperature_i = rx_data[6] << 8 | rx_data[7];
		
		cybergear_measure.angle = cybergear_measure.angle_i * angle_param ;
		cybergear_measure.gyro = cybergear_measure.gyro_i * spd_param ;
		cybergear_measure.torque = cybergear_measure.torque_i* torque_param ;
		cybergear_measure.temp = cybergear_measure.temperature_i * temp_param;
	}

}
//获得编码器数据 
static void get_encoder_measure(motor_encoder_measure_t* motor_encoder , uint8_t rx_data[8])
{
   motor_encoder->encoder_id = rx_data[0];
   motor_encoder->positoin_i  =(uint16_t)(rx_data[1]<<8|rx_data[2]) - 32768;
	 motor_encoder->speed_i = (uint16_t)((rx_data[3]<<4|rx_data[4]>>4)) - 2048;
	 motor_encoder->torque_i = (uint16_t)(((rx_data[4]<<8|rx_data[5]))&0xfff) - 2048;
}

const motor_measure_t *get_fric1_motor_measure_point(void)
{
    return &motor_gimbal[0];
}

const motor_measure_t *get_fric2_motor_measure_point(void)
{
    return &motor_gimbal[1];
}

const motor_measure_t *get_fric3_motor_measure_point(void)
{
    return &motor_gimbal[2];
}

const motor_measure_t *get_fric4_motor_measure_point(void)
{
    return &motor_gimbal[3];
}

const motor_measure_t *get_trigger1_motor_measure_point(void)
{
	return &motor_gimbal[5];
}


const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
	return &motor_gimbal[6];
}

const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
	return &motor_gimbal[7];
	
}






