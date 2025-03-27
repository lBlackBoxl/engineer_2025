#ifndef NX_COMMUNICATE_H
#define NX_COMMUNICATE_H

#include "main.h"

void nx_communicate_task(void const *pvParameters);

extern float txData[6];
extern float rxData[6];

#endif
