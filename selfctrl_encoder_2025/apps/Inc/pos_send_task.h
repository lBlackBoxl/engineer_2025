#ifndef POS_SEND_TASK_H
#define POS_SEND_TASK_H

//lift real 470mm & small 156mm
//arm3508 real 660mm & small 220mm

#include "global_defines.h"

#define ENCODER_CYCLE_RESOLUTION					32768
#define ENCODER_HALF_CYCLE_RESOLUTION 		ENCODER_CYCLE_RESOLUTION/2


#define LIFT_RATIO			0.8f			//Ì§ÉýÓ³ÉäÏµÊý
#define ARM_3508_RATIO	PI				//ºáÒÆÓ³ÉäÏµÊý

#define JOINT_0_RATIO				0.8f
#define JOINT_0_OFFSET			0
#define JOINT_0_MINPOS			-2.85f
#define JOINT_0_MAXPOS			2.90f

#define JOINT_1_RATIO				PI
#define JOINT_1_OFFSET			0
#define JOINT_1_MINPOS      -3.129f
#define JOINT_1_MAXPOS			-0.329f

#define JOINT_2_RATIO				1
#define JOINT_2_OFFSET			0
#define JOINT_2_MINPOS     	-1.242f
#define JOINT_2_MAXPOS			1.558f

#define JOINT_3_RATIO				1
#define JOINT_3_OFFSET			0
#define JOINT_3_MINPOS     	-1.57f
#define JOINT_3_MAXPOS			1.57f

#define JOINT_4_RATIO				-3.46f/PI  
#define JOINT_4_OFFSET			0
#define JOINT_4_MINPOS     	-1.7f
#define JOINT_4_MAXPOS			1.7f

#define JOINT_5_RATIO				1.27f
#define JOINT_5_OFFSET			0
#define JOINT_5_MINPOS     	-PI
#define JOINT_5_MAXPOS			PI

#define JOINT_6_RATIO				1
#define JOINT_6_OFFSET			0
#define JOINT_6_MINPOS     	-65535
#define JOINT_6_MAXPOS			65535

#endif
