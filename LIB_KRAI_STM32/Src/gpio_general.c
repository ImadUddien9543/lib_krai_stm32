/*
 * gpio_general.c
 *
 *  Created on: Sep 12, 2023
 *      Author: Imaduddien
 */

#include "gpio_general.h"

GPIO_Struct piston_a = {.gpio_x = PISTON_A_GPIO_Port, .pin_num = PISTON_A_Pin};

void HIGH(GPIO_Struct *gp){
	HAL_GPIO_WritePin(gp->gpio_x, gp->pin_num, GPIO_PIN_SET);
}

void LOW(GPIO_Struct *gp){
	HAL_GPIO_WritePin(gp->gpio_x, gp->pin_num, GPIO_PIN_RESET);
}

GPIO_PinState READ(GPIO_Struct *gp){
	return HAL_GPIO_ReadPin(gp->gpio_x, gp->pin_num);
}

void TOGGLE(GPIO_Struct *gp, uint32_t delay){
	gp->start = HAL_GetTick();
	gp->dt = gp->start - gp->last;
	if(gp->dt >= delay){
		HAL_GPIO_TogglePin(gp->gpio_x, gp->pin_num);
		gp->last = gp->start;
	}
	else __NOP();
}
