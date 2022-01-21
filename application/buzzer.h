#ifndef _BUZZER_H_
#define _BUZZER_H_


#include "main.h"
#include "tim.h"

void buzzer_on(uint16_t psc, uint16_t pwm);
void buzzer_off(void);
void buzzer_start(void);
void buzzer_error(void);

#endif
