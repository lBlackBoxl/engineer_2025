#ifndef GLOBALDEFINES_H
#define GLOBALDEFINES_H

//Program Switch
#define REMOTE_CONTROL_NEW_ENABLE 	0																//新遥控器1 老遥控器0
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
	*@brief:限幅函数，若value大于max则输出max，若小于min则输出min，否则输出value
	*@param;value 输入值
	*@param:min   下限
	*@param:max		上限
	*@retval:value经过限幅后的值
  */
#ifndef DEADBAND_LIMIT
#define DEADBAND_LIMIT(value, min, max)   ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))
#endif



#endif
