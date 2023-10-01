/*
 * read_RPM.c
 *
 *  Created on: Feb 4, 2023
 *      Author: udin
 */

#include "QEI.h"

//contoh saja
QEI OMNI_BASE = {
		.sample_time = 100, .pulse = 0, .pulse_ = 0,
		.port_A = ENC3_A_GPIO_Port, .port_B = ENC3_B_GPIO_Port,
		.pin_A = ENC3_A_Pin, .pin_B = ENC3_B_Pin,
		.ppr = 200, .gear_ratio = 19.2
};

QEI ENC_EXT_1 = {
		.sample_time = 100, .pulse = 0, .pulse_ = 0,
		.port_A = ENC3_A_GPIO_Port, .port_B = ENC3_B_GPIO_Port,
		.pin_A = ENC3_A_Pin, .pin_B = ENC3_B_Pin,
		.r = 0.1, .ppr = 360
};

QEI MEKANISME = {
		.sample_time = 100, .pulse = 0, .pulse_ = 0,
		.port_A = ENC3_A_GPIO_Port, .port_B = ENC3_B_GPIO_Port,
		.pin_A = ENC3_A_Pin, .pin_B = ENC3_B_Pin,
		.ppr = 14, .gear_ratio = 1320 //(1 : 264 motor mendrive 1 : 5 di mekanisme)
};

QEI ENC_EXT_TIM_1 = {
		.sample_time = 100, .pulse = 0, .pulse_ = 0,
		.tim = &htim5,
		.r = 0.1, .ppr = 360
};

void get_RPM(QEI *q){
	q->start_time = HAL_GetTick();
	q->dt = q->start_time - q->prev_time;
	if(q->dt >= q->sample_time){
		q->RPM = (q->pulse * 60.0f * q->sample_time) / (q->ppr * q->gear_ratio * 2);
		q->pulse = 0;
		q->prev_time = q->start_time;
	}
}
void get_RAD_S(QEI *q){
	q->start_time = HAL_GetTick();
	q->dt = q->start_time - q->prev_time;
	if(q->dt >= q->sample_time){
		q->RPM = (q->pulse * M_TWOPI * q->sample_time) / (q->ppr * q->gear_ratio * 2);
		q->pulse = 0;
		q->prev_time = q->start_time;
	}
}
void get_MTR_S(QEI *q){
	q->start_time = HAL_GetTick();
	q->dt = q->start_time - q->prev_time;
	if(q->dt >= q->sample_time){
		q->RPM = (q->pulse * M_TWOPI * q->r * q->sample_time) / (q->ppr * q->gear_ratio * 2);
		q->pulse = 0;
		q->prev_time = q->start_time;
	}
}

void get_DEG_S(QEI *q){
	q->start_time = HAL_GetTick();
	q->dt = q->start_time - q->prev_time;
	if(q->dt >= q->sample_time){
		q->RPM = (q->pulse * 360.0f * q->sample_time) / q->denom;
		q->pulse = 0;
		q->prev_time = q->start_time;
	}
}

/*
 * hitung semua gear ratio yg ada di satu mekanisme
 * jika ada belt/rantai hitung perbandinganya juga
 * misal:
 * pg45 1:264 ----- driven gear:drive gear = 2:10 = 1:5
 * maka .gear_ratio = 264 * 5;
 * note: lebih baik gunakan encoder terpisah (lgsg di mekanisme) utk menghindari backlash
 * jika encoder lgsg di as/shaft mekanisme, gear_ratio ditulis 1
 * */

void get_DEG(QEI *q){ //untuk mekanisme pan-tilt atau angular spt lengan
	q->ANG_DEG = (q->pulse_ * 360.0f) / (2 * q->ppr * q->gear_ratio);
}

void get_RAD(QEI *q){ //untuk mekanisme pan-tilt atau angular spt lengan
	q->ANG_RAD = (q->pulse_ * M_TWOPI) / (2 * q->ppr * q->gear_ratio);
}

void get_MTR(QEI *q){ //untuk mencari jarak dari odometry / enc external
	q->DIST_M = (q->pulse_ * q->r * M_TWOPI) / (2 * q->ppr);
}

void TIM_RPM(QEI *q){
	q->start_time = HAL_GetTick();
	q->dt = q->start_time - q->prev_time;
	if(q->dt >= q->sample_time){
		q->RPM = (__HAL_TIM_GET_COUNTER(q->tim) * q->sample_time * 60.0f) / (q->ppr * q->gear_ratio * 2);
		__HAL_TIM_SET_COUNTER(q->tim, 0);
		q->prev_time = q->start_time;
	}
}

void pin_A(QEI *q){
	if((HAL_GPIO_ReadPin(q->port_B, q->pin_B)) == GPIO_PIN_RESET){
		q->pulse++; q->pulse_++;
	}
	else{
		q->pulse--; q->pulse_--;
	}
}
void pin_B(QEI *q){
	if((HAL_GPIO_ReadPin(q->port_A, q->pin_A)) == GPIO_PIN_RESET){
		q->pulse--; q->pulse_--;
	}
	else{
		q->pulse++; q->pulse_++;
	}
}
