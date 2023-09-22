/*
 * QEI.h
 *
 *  Created on: Sep 5, 2023
 *      Author: Imaduddien
 */

#ifndef INC_QEI_H_
#define INC_QEI_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "main.h"
#include "math.h"
#include "stdint.h"

/*
 * HOW to interface
 *
 *
 * untuk write pin:
 * setup pin dulu GPIOx dan Pin_Num nya
 * invoke nya:
 * 		write_pin(&struct_nya, state);
 *
 * read RPM untuk fungsi INT_RPM dan INT_RPM2:
 * 	di dalam:
 *
 * 	kalo ga nambah pulse nya, coba tuker pin nya di member struct RPM pin_enc_x dan port_enc_x
 *
 *
 *	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
 *		if(GPIO_Pin == struct_nya.pin_enc_1 || GPIO_Pin == struct_nya.pin_enc_2) read_pulse(&struct_nya);
 *		else __NOP(); //https://stackoverflow.com/questions/24359632/how-does-asmnop-works
 *	}
 *
 *	encoding pembacaan encoder ada 2: half-quadrature sma quadrature:
 *	intinya, half-quadrature baca rising/falling pulse dari encoder
 *	klo quadrature rising dan falling
 *
 *	config nya tgl invoke set_encoding() di main bawahnya *USER CODE BEGIN 2*
 *	yg penting setelah MX_GPIO_INIT();
 *
 *	stlh itu, di loop panggil fungsi baca_rpm() e.g. INT_RPM(), INT_RPM2(), INT_RPM3()
 *	=================================================================================
 *
 *	untuk servo:
 *	buat baca mekanisme kontrol posisi: sudut (angular type e.g. pan/tilt) & jarak (linear type e.g. rack pinion/lead screw)
 *	di callback interrupt:
 *
 *	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
 *		if(GPIO_Pin == struct.pin_enc_1 || GPIO_Pin == struct.pin_enc_2) read_servo_pulse;
 *		else __NOP(); //satu saja dlm satu callback bersama
 *	}
 * */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	X2R = 2,
	X2F = 2,
	X4 = 4
}ENCODING;

typedef struct __attribute__((__packed__)) RPM_struct {
	uint32_t start_time, prev_time, update_s, d_time, RPM_U;
	double pulse_enc, pulse_dist, pulse_tim, pulse_enc_last;
	double RPM, d_wheel, ppr, dist, gear_ratio;
	GPIO_TypeDef *port_enc_1, *port_enc_2;
	GPIO_InitTypeDef *port_mode;
	uint16_t pin_enc_1, pin_enc_2;
	ENCODING X;
} RPM;

typedef struct __attribute__((__packed__)) _servo_ {
	uint32_t start_time, prev_time, sample_time;
	int pulse_enc, pulse_rpm;
	double RPM, ppr, gear_ratio;
	GPIO_TypeDef *port_enc_1, *port_enc_2;
	GPIO_InitTypeDef *port_mode;
	ENCODING X;
	uint16_t pin_enc_1, pin_enc_2;
} servo;

extern RPM w1, w2, RB;

void rpm_encoding(RPM *rpm, ENCODING X);
void INT_RPM(RPM *rpm);
void INT_RPM2(RPM *rpm);
void INT_RPM3(RPM *rpm);
void TIM_RPM(RPM *rpm, TIM_HandleTypeDef *encoder_cnt);
void read_pulse(RPM *rpm);
void enc_dist(RPM *rpm);

void servo_encoding(servo *s, ENCODING X);
void read_servo_pulse(servo *s);
float get_angle(servo *s);

#ifdef __cplusplus
}
#endif



#endif /* INC_QEI_H_ */
