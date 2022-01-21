#include "buzzer.h"



void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);

}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}

void buzzer_start()
{
	buzzer_on(45, 45);
	HAL_Delay(100);
	buzzer_on(40, 40);
	HAL_Delay(100);
	buzzer_on(35, 35);
	HAL_Delay(100);
	buzzer_off();
}

void buzzer_error()
{
	buzzer_on(60, 60);
	HAL_Delay(100);
	buzzer_on(30, 30);
	HAL_Delay(100);
}
