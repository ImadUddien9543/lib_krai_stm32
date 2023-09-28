/*
 * read_RPM.c
 *
 *  Created on: Feb 4, 2023
 *      Author: udin
 */

#include "QEI.h"

void rpm_encoding(RPM *rpm, ENCODING X){
	if(X == X2R)		rpm->port_mode->Mode = GPIO_MODE_IT_RISING;
	else if(X == X2F)	rpm->port_mode->Mode = GPIO_MODE_IT_FALLING;
	else if(X == X4)	rpm->port_mode->Mode = GPIO_MODE_IT_RISING_FALLING;
	else __NOP();
	HAL_GPIO_Init(rpm->port_enc_1, rpm->port_mode);
	HAL_GPIO_Init(rpm->port_enc_2, rpm->port_mode);
}

void INT_RPM(RPM *rpm){
	rpm->start_time = HAL_GetTick();
	if((rpm->start_time - rpm->prev_time) >= (rpm->update_s)){
		rpm->prev_time = rpm->start_time;
		rpm->RPM = ((rpm->pulse_enc - rpm->pulse_enc_last) * 60.0f * rpm->update_s) / (rpm->ppr * rpm->X * rpm->gear_ratio);
		rpm->pulse_enc_last = rpm->pulse_enc;
	}
}

void INT_RPM2(RPM *rpm){ //no gear_ratio (tidak dicouple gearbox, pulley, sprocket, dll)
	rpm->start_time = HAL_GetTick();
	rpm->d_time = rpm->start_time - rpm->prev_time;
	int t_ = 1000 / rpm->update_s;
	if(rpm->d_time >= (t_)){
		rpm->RPM = (rpm->pulse_enc * 60.0f * rpm->update_s) / (rpm->ppr * rpm->X);
		rpm->pulse_enc = 0;
		rpm->prev_time = rpm->start_time;
	}
}

void INT_RPM3(RPM *rpm){ // dicouple gearbox, pulley, sprocket, dll)
	rpm->start_time = HAL_GetTick();
	rpm->d_time = rpm->start_time - rpm->prev_time;
	int t_ = 1000 / rpm->update_s;
	if(rpm->d_time >= (t_)){
		rpm->RPM = (rpm->pulse_enc * 60.0f * rpm->update_s) / (rpm->ppr * rpm->X * rpm->gear_ratio);
		rpm->pulse_enc = 0;
		rpm->prev_time = rpm->start_time;
	}
}

void TIM_RPM(RPM *rpm, TIM_HandleTypeDef *encoder_cnt){
	rpm->start_time = HAL_GetTick();
	if((rpm->start_time - rpm->prev_time) >= (1000 / rpm->update_s)){
		rpm->prev_time = rpm->start_time;
		rpm->pulse_tim = __HAL_TIM_GET_COUNTER(encoder_cnt);
		rpm->RPM_U = (rpm->pulse_tim * rpm->update_s * 60) / rpm->ppr * 2;;
		__HAL_TIM_SET_COUNTER(encoder_cnt, 0);
	}
}

void read_pulse(RPM *rpm){
	if((HAL_GPIO_ReadPin(rpm->port_enc_1, rpm->pin_enc_1)) == GPIO_PIN_RESET){
		rpm->pulse_enc--;
		rpm->pulse_dist--;
	}
	else if((HAL_GPIO_ReadPin(rpm->port_enc_2, rpm->pin_enc_2)) == GPIO_PIN_RESET){
		rpm->pulse_enc++;
		rpm->pulse_dist++;
	}
	else __NOP();
}

void enc_dist(RPM *rpm){
	rpm->dist = (rpm->pulse_dist * rpm->d_wheel * M_PI) / (rpm->ppr * rpm->X);
}

void servo_encoding(servo *s, ENCODING X){
	if(X == X2R)		s->port_mode->Mode = GPIO_MODE_IT_RISING;
	else if(X == X2F)	s->port_mode->Mode = GPIO_MODE_IT_FALLING;
	else if(X == X4)	s->port_mode->Mode = GPIO_MODE_IT_RISING_FALLING;
	else __NOP();
	HAL_GPIO_Init(s->port_enc_1, s->port_mode);
	HAL_GPIO_Init(s->port_enc_2, s->port_mode);
}

void read_servo_pulse(servo *s){
	if((HAL_GPIO_ReadPin(s->port_enc_1, s->pin_enc_1)) == GPIO_PIN_RESET){
		s->pulse_enc++;

	}
	else if ((HAL_GPIO_ReadPin(s->port_enc_2, s->pin_enc_2)) == GPIO_PIN_RESET){
		s->pulse_enc--;
	}
	else __NOP();
}

/*
 * hitung semua gear ratio yg ada di satu mekanisme
 * jika ada belt/rantai hitung perbandinganya juga
 * misal:
 * pg45 1:264 ----- driven gear:drive gear = 2:10 = 1:5
 * maka .gear_ratio = 264 * 5;
 * note: lebih baik gunakan encoder terpisah (lgsg di mekanisme) utk menghindari backlash
 * jika encoder lgsg di as/shaft mekanisme, gear_ratio ditulis 1
 *
 */
float get_deg(servo *s){
	return (s->pulse_enc * 360.0f) / (s->gear_ratio * s->ppr * s->X);
}

void servo_deg_s(servo *s){
	s->start_time = HAL_GetTick();
	s->dt = s->start_time - s->prev_time;
	if(s->dt >= s->sample_time){
		s->prev_time = s->start_time;
		s->deg_s =	(s->pulse_enc - s->pulse_enc_) * 360.0f * s->sample_time / (s->ppr * s->X * s->gear_ratio);
		s->pulse_enc_ = s->pulse_enc;
	}
}
