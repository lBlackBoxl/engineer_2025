#ifndef POS_SEND_TASK_H
#define POS_SEND_TASK_H

//lift real 470mm & small 156mm
//arm3508 real 660mm & small 220mm

#include "global_defines.h"

#define ENCODER_CYCLE_RESOLUTION					32768
#define ENCODER_HALF_CYCLE_RESOLUTION 		ENCODER_CYCLE_RESOLUTION/2


#define LIFT_RATIO			0.8f			//Ì§ÉýÓ³ÉäÏµÊý
#define ARM_3508_RATIO	PI				//ºáÒÆÓ³ÉäÏµÊý

#define JOINT_0_RATIO				1.0f
#define JOINT_0_OFFSET			-0.153f
#define JOINT_0_MINPOS			-2.85f
#define JOINT_0_MAXPOS			2.90f

#define JOINT_1_RATIO				1.0f
#define JOINT_1_OFFSET			0
#define JOINT_1_MINPOS      -0.01f
#define JOINT_1_MAXPOS			2.85f

#define JOINT_2_RATIO				1
#define JOINT_2_OFFSET			0
#define JOINT_2_MINPOS     	-0.01f
#define JOINT_2_MAXPOS			2.8f

#define JOINT_3_RATIO				1
#define JOINT_3_OFFSET			0
#define JOINT_3_MINPOS     	-2.03f
#define JOINT_3_MAXPOS			2.03f

#define JOINT_4_RATIO				1.12f
#define JOINT_4_OFFSET			-0.674f
#define JOINT_4_MINPOS     	0.06f
#define JOINT_4_MAXPOS			3.62f

#define JOINT_5_RATIO				1.27f
#define JOINT_5_OFFSET			0
#define JOINT_5_MINPOS     	-65536
#define JOINT_5_MAXPOS			65535


#endif
