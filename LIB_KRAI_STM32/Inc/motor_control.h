/*
 * motor_control.h
 *
 *  Created on: Jan 26, 2023
 *      Author: udin
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"
#include "main.h"
#include "stdint.h"
#include "math_formula.h"
#include "stdlib.h"
#include "stdbool.h"
#include "gpio_general.h"


#ifdef __cplusplus
extern "C" {
#endif

/*
 *
 * Freq = APB_x / DIV
 * Freq = APB_x / (ARR+1) * (PSC+1)  (ditambah 1 dari register sana nya utk mencegah division by zero)
 * 									 (nanti di ioc di kurangi 1)
 *
 * ARR 16-bit timer = 1 < ARR < 0xffff (65.535)
 * ARR 32-bit timer = 1 < ARR < 0xffff ffff (4.294.967.295)
 *
 * misal:
 *
 * 16-bit timer: misal 16-bit timer tp APB_1
 * 		sysclock = 180MHz (PLL_CLK)
 * 		freq	= 50Hz
 * 		DIV		= APB_1 / 50
 * 				= 90MHz / 50Hz
 * 				= 1.8kHZ
 *
 * 		ARR max yang boleh dicapai = 65.535 (biar bulat ARR jadi 60000)
 * 		PSC		= 1.8kHz / 60000
 * 				= 30
 *
 * 		maka, ARR = 60000, PSC = 30
 *
 * 32-bit timer: misal 32-bit timer tp APB_2
 * 		sysclock = 180MHz (PLL_CLK)
 * 		freq	= 50Hz
 * 		DIV		= APB_2 / 50
 * 				= 180MHz / 50Hz
 * 				= 3.6kHZ
 *
 * 		ARR max yang boleh dicapai = 4.294.967.295 (lebih dari DIV, jadi ARR max = DIV)
 * 		PSC		= 3.6kHz / 3.6k
 * 				= 1
 *
 * 		maka, ARR = 3.600.000, PSC = 1 (disini digit ARR sgt byk, cari nilai yg masuk akal saja)
 *
 * Semakin besar ARR semakin you have control over pwm (depends on what APB you're using (dan ESC))
 * ----------Sesuaikan dengan APBx Timer Clock-------------
 *
 * period microsecond pulsa bldc/servo:
 * 	1/freq_total = 1/50Hz = 20ms
 * 	range normal servo 1 ms - 2 ms ---> duty = 1/20 - 2/20 = 0.05 - 0.1
 * 	range rata2 1100 us - 1900 us utk esc atau 1.1 ms - 1.94 ms
 *
 * 	klo pake esc atau servo BACA MANUAL dulu, liat min dan max periode yg didukung
 * 	misal esc yg di roller 1100 us - 1940 us (ada di manual)
 *
 *
 */

typedef struct motor_channel_struct {
	TIM_HandleTypeDef* in1_;
	TIM_HandleTypeDef* in2_;
	uint32_t ch1_;
	uint32_t ch2_;
	GPIO_TypeDef *en_port;
	uint16_t en_pin;
} motor_channel;

typedef struct esc_struct{
	TIM_HandleTypeDef* htim;
	uint32_t channel;
	float min_ccr, max_ccr;
	float min_us, max_us;
} esc;

extern esc bldc_1, bldc_2;
extern motor_channel LeftFront, LeftBack, RightFront, RightBack, X_penembak, Y_penembak, Chain_lift, Chain_lift2;

void motor_init(motor_channel *wheel_n);
void motor_drive(motor_channel *wheel_n, int_fast16_t *rpm);
void bldc_init(esc *servo);
void bldc_drive(esc *servo, uint_fast32_t *duty);
void bldc_duty(esc *servo, float *percent);	//0.0  - 100.0
void disable_motor(motor_channel *wheel_n);
void enable_motor(motor_channel *wheel_n);

#ifdef __cplusplus
}
#endif

#endif /* INC_MOTOR_CONTROL_H_ */
