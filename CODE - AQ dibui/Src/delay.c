/*
 * delay.c
 *
 *  Created on: Dec 6, 2021
 *      Author: manht
 */
#include "delay.h"

extern TIM_HandleTypeDef htim4;
void delay_us(uint32_t us){
	__HAL_TIM_SET_COUNTER(&htim4,0);
	HAL_TIM_Base_Start(&htim4);
	while(__HAL_TIM_GET_COUNTER(&htim4) < us);
	HAL_TIM_Base_Stop(&htim4);
}
