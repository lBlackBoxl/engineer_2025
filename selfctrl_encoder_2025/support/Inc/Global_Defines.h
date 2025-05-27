#ifndef GLOBALDEFINES_H
#define GLOBALDEFINES_H

//Program Switch
#define REMOTE_CONTROL_NEW_ENABLE 	0																//��ң����1 ��ң����0
#define REMOTE_CONTROL_OLD_ENABLE 	(!REMOTE_CONTROL_NEW_ENABLE)


//global consts
#ifndef PI
#define PI 					3.14159265358979323846f
#endif

#ifndef USER_NULL
#define USER_NULL (void*)0
#endif



//global functions

/**
	*@brief:�޷���������value����max�����max����С��min�����min���������value
	*@param;value ����ֵ
	*@param:min   ����
	*@param:max		����
	*@retval:value�����޷����ֵ
  */
#ifndef DEADBAND_LIMIT
#define DEADBAND_LIMIT(value, min, max)   ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))
#endif



#endif
