#ifndef __BUZZER_H
#define __BUZZER_H
#include "struct_typedef.h"

extern void buzzer_int(void);
extern void buzzer_on(uint16_t psc, uint16_t pwm);
extern void buzzer_off(void);

#endif
