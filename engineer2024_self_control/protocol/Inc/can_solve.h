#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H
#include "stdint.h"
#include "arm_math.h"


#define ENCODER_MASTER_ID 100
#define CAN_ENCODER_ID 1
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -12.0f
#define T_MAX 12.0f

#define iq_index 0x7006
#define spd_index 0x700A
#define loc_index 0x7016
#define angle_param 0.0003835069007
#define spd_param 0.0009155552843
#define torque_param 0.0003662221137
#define temp_param 0.1
struct Tx_exCanIdInfo{
	uint32_t id:8;
	uint32_t data:16;
	uint32_t mode:5;
	uint32_t res:3;
};

typedef enum
{
	move_mode = 0,
	position_mode = 1,
	speed_mode = 2,
	current_mode = 3,
}cybergear_mode;

union Tx_exId_uint32
{
	struct Tx_exCanIdInfo exid;
	uint32_t aa;
};

struct Rx_exCanIdInfo{
	uint32_t id:8;
	uint32_t motor_CanId:8;
	uint32_t undervoltage_err:1;
	uint32_t overcurrent_err:1;
	uint32_t hightemperature_err:1;
	uint32_t magnetic_coding_err:1;
	uint32_t hall_coding_err:1;
	uint32_t calibration_err:1;
	uint32_t mode:2;
	uint32_t communication_mode:5;
	uint32_t res:3;

};
union Rx_exId_uint32
{
	struct Rx_exCanIdInfo exid;
	uint32_t aa;
};


typedef enum
{
    CAN_FRIC1_MOTOR_ID 		= 0x201,
    CAN_FRIC2_MOTOR_ID 		= 0x202,
	CAN_FRIC3_MOTOR_ID 		= 0x203,
	CAN_FRIC4_MOTOR_ID 		= 0x204,
	CAN_TRIGGER1_MOTOR_ID	= 0x206,
	CAN_YAW_MOTOR_ID 		= 0x209,
	//pitch为can1，trigge1为can2，不冲突//准备把pitch改为can2，trigger改为can1，pitch还没改
    CAN_PIT_MOTOR_ID 		= 0x205,
    
	
    CAN_SHOOT_ALL_ID 		= 0x200,
	CAN_TRIGGER_ALL_ID      = 0x1FF,
    CAN_PITCH_ALL_ID 		= 0x1FF,
    CAN_YAW_ALL_ID			= 0x2FF,
	CAN_STATE_ID            = 0x210,
	CAN_SEND_GYRO_ID        = 0x220,
	CAN_SEND_SPEED_ID       = 0X221,
    CAN_ANGLE_ID            = 0x231,
    CAN_REFEREE_ID 			= 0x230,
	CAN_HIT_ID              = 0x235,

	CyberGear_CAN_ID        = 0x00,
	CyberGear_CAN_MASTER_ID = 0x7F,
} can_msg_id_e;

typedef struct 
{
    uint16_t ecd;
    int16_t  speed_rpm;
    int16_t  given_current;
    uint8_t  temperate;
    int16_t  last_ecd;
} motor_measure_t;

union FloatUint8_t
{
    float32_t a;
    uint8_t b[4];
};
extern int8_t gimbal_flag;
extern uint32_t timeout;
extern uint8_t color_flag;
extern float32_t bullet_speed;
extern uint8_t position_flag;
extern uint8_t FIRE_FLAG;
extern uint8_t STOP_FLAG;

extern void CAN_cmd_pitch(int16_t current1,int16_t trigger_current);
extern void CAN_cmd_yaw(int16_t current1);
extern void CAN_cmd_shoot(int16_t current1,int16_t current2, int16_t current3, int16_t current4);
extern void CAN_cmd_trigger(int16_t current1, int16_t current2);
extern void CAN_cmd_communication(void);
extern void CAN_cmd_vel(vel_rx_data_t *vel_data); 
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_trigger1_motor_measure_point(void);
extern const motor_measure_t *get_trigger2_motor_measure_point(void);
extern const motor_measure_t *get_fric1_motor_measure_point(void);
extern const motor_measure_t *get_fric2_motor_measure_point(void);
extern const motor_measure_t *get_fric3_motor_measure_point(void);
extern const motor_measure_t *get_fric4_motor_measure_point(void);
extern void CAN_cmd_state(float32_t angle);
extern void CAN_send_gyro(float32_t gimbal_gyro);
extern void CAN_send_Speed(vel_rx_data_t *command);
extern void Can_crbergear_runmode_set(uint8_t id, uint16_t master_id, uint8_t runmode);
extern void Can_cybergear_enable(uint8_t id, uint16_t master_id);
extern void Can_cybergear_param_write(uint8_t id, uint16_t master_id, uint16_t index, float32_t param_ref);
extern void Can_cybergear_stop(uint8_t id,uint16_t master_id);
extern void Can_encoder_stop(void);
extern void Can_encoder_set(void);
extern void Can_encoder_current_mode(int16_t current);
extern void Can_encoder_positoin_mode(void);
extern void Can_encoder_start(void);


#endif

