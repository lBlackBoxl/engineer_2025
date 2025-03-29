#ifndef NX_COMMUNICATE_H
#define NX_COMMUNICATE_H

#include "main.h"

void nx_communicate_task(void const *pvParameters);

extern float txData[6];

typedef struct
{
	long long time_flag;
	long long time_last_flag;
	double rxData[6];
	uint8_t rxData_temp[56];
} rxData_t;

extern rxData_t nx_rxData;
#endif
