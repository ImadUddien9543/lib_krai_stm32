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
#include "stdlib.h"

/*
 * HOW to interface
 *
 * 	note : kalo ga nambah pulse nya, coba tuker pin nya di member struct RPM pin_enc_x dan port_enc_x
 *
 *	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
 *		if(GPIO_Pin == OMNI_BASE.pin_A) pin_A(&OMNI_BASE);
 *		if(GPIO_Pin == OMNI_BASE.pin_B) pin_B(&OMNI_BASE);
 *		else __NOP();
 *	}
 *
 *	stlh itu, di loop panggil fungsi baca_rpm()
 *
 * INGAT:
 *  2pi rad/s  = 1 Hz = 60 rpm
 *  jika tidak ada gearbox, gear drive, belt drive, chain drive dll, gear_ratio tulis 1 (https://www.youtube.com/watch?app=desktop&v=6irF1P7cCJk)
 * */


#ifdef __cplusplus
extern "C" {
#endif

typedef struct QEI_Struct {
	uint32_t start_time, prev_time, sample_time, dt; //dalam ms
	float pulse; //untuk besaran kecepatan
	float pulse_; // untuk besaran jarak
	/*	r = jari-jari roda (untuk odometry)
	 * 	gear_ratio = motor gearbox, geardrive, beltdrive dll
	 * */
	float r, ppr, gear_ratio;
	float RPM, RAD_S, M_S, DEG_S;
	float DIST_M, ANG_DEG, ANG_RAD;
	float denom;
	//interrupt
	GPIO_TypeDef *port_A, *port_B;
	uint16_t pin_A, pin_B;
	//timer_enc
	TIM_HandleTypeDef *tim;
} QEI;

extern QEI OMNI_BASE, EXT_ENC_1, MEKANISME, EXT_ENC_TIM_1; //contoh
extern TIM_HandleTypeDef htim5;

//khusus interrupt taro di callback interrupt
//rising edge interrupt
void pin_A(QEI *q);
void pin_B(QEI *q);

//loop function
void get_RPM(QEI *q);
void get_RAD_S(QEI *q);
void get_MTR_S(QEI *q);
void get_DEG_S(QEI *q);

void get_DEG(QEI *q);
void get_MTR(QEI *q);
void get_RAD(QEI *q);

void TIM_RPM(QEI *q);

#ifdef __cplusplus
}
#endif



#endif /* INC_QEI_H_ */
