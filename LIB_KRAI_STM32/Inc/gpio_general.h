/*
 * gpio_general.h
 *
 *  Created on: Sep 12, 2023
 *      Author: Imaduddien
 */

#ifndef INC_GPIO_GENERAL_H_
#define INC_GPIO_GENERAL_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "main.h"
#include "math.h"
#include "stdint.h"

/*
untuk write pin:
 * setup pin dulu GPIOx dan Pin_Num nya
 * invoke nya:
 * 		pin_set(&struct_nya);
 *
*/

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _GPIO_Struct_ {
    GPIO_TypeDef *gpio_x;
    uint16_t pin_num;
    GPIO_PinState state;
    uint32_t start, last, dt;
} GPIO_Struct;

extern GPIO_Struct piston_a;

void HIGH(GPIO_Struct *gp);
void LOW(GPIO_Struct *gp);
void TOGGLE(GPIO_Struct *gp, uint32_t delay);
GPIO_PinState READ(GPIO_Struct *gp);

#ifdef __cplusplus
}
#endif


#endif /* INC_GPIO_GENERAL_H_ */
