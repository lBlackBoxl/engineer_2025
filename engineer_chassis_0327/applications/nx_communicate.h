#ifndef NX_COMMUNICATE_H
#define NX_COMMUNICATE_H

#include "main.h"

void nx_communicate_task(void const *pvParameters);

extern float txData[6];

typedef struct
{
	float time_flag;
	float rxData[6];
	uint8_t rxData_temp[28];
} rxData_t;

extern rxData_t nx_rxData;
#endif
