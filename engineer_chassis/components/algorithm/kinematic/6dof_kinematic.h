/**
  ****************************(C) COPYRIGHT 2023 TJU****************************
  * @file       6dof_kinematic.c/h
  * @brief      Kinematics solution of robotic arm task,
  *             机械臂运动学解算任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-20-2023     王艺文              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 TJU****************************
  */
#ifndef DOF6_KINEMATIC_SOLVER_H
#define DOF6_KINEMATIC_SOLVER_H


#include "arm_math.h"
#include "math.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdbool.h"

#ifndef user_malloc
	#ifdef _CMSIS_OS_H
		#define user_malloc pvPortMalloc
	#else
		#define user_malloc malloc
	#endif
#endif

#define mat arm_matrix_instance_f32
#define Matrix_Init     arm_mat_init_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Inverse arm_mat_inverse_f32

#define RAD_TO_DEG  57.29577951308232088f


typedef struct 
{
		float theta[6];
}Joint6D_t;

typedef struct 
{
		float theta[8][6];//逆解算的八组解
}Solver6D_t;

typedef struct 
{
    float X, Y, Z;
    float A, B, C;//A-yaw,B-pitch,C-roll
		float Q[4];
}Pose6D_t;

typedef struct
{
		float _alpha[6];
		float _a[6];
	  float _d[6];
	  int8_t MatStatus;
	
		mat _T[6];
		float *_T_data[6];
		mat _R06;
		float *_R06_data;
		mat R02_inv;
		float *_R02_inv_data;
		mat R23_inv;
		float *_R23_inv_data;
		mat _R36;
		float *_R36_data;
		mat _P56;
		float *_P56_data;
		mat _P56_temp;
		float *_P56_temp_data;
	
		mat fk_temp_matrix[2];
		float *fk_temp_matrix_data[2];		
	
		mat _ik_temp_matrix;
		float *_ik_temp_matrix_data;
	
		float _IK_Flags[8][4];
	
} Robotic_6DOF;


void Joint6D_Init(Robotic_6DOF * Robotic_6D);

void SolveFK(Robotic_6DOF * Robotic_6D, const Joint6D_t *_Joint6D, Pose6D_t *_Pose6D);

bool SolveIK(Robotic_6DOF * Robotic_6D, Pose6D_t *_inputPose6D, Solver6D_t *_Out_Solver6D, const Joint6D_t *_lastJoint6D, uint8_t _Quaterniont_mode);


#endif //DOF6_KINEMATIC_SOLVER_H
