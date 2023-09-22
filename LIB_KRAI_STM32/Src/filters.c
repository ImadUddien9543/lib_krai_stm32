/*
 * filters.c
 *
 *  Created on: Sep 10, 2023
 *      Author: Imaduddien
 */

#include "filters.h"


float low_pass(filter *f, float input){
	f->start = HAL_GetTick();
	f->dt_ = (f->start - f->last) * 1e-03f; // second
	if(f->dt_ >= f->ts_){
		f->prev_out = f->out;
		f->last = f->start;
	}

	f->alpha = f->dt_ / (f->RC + f->dt_) * 1.0f;
	//dicoba kedua hasilnya
	//	f->out = (f->alpha * input) + ((1.0f - f->alpha) * f->prev_out);
	f->out = f->prev_out + (f->alpha * (input - f->prev_out));//code refactored ver
	return f->out / 1.0f;
}

float hi_pass(filter *f, float input){
	f->start = HAL_GetTick();
	f->dt_ = (f->start - f->last) * 1e-03f; //second
	if(f->dt_ >= f->ts_){
		f->prev_out = f->out;
		f->prev_in = input;
		f->last = f->start;
	}
	f->alpha = f->RC / (f->RC + f->dt) * 1.0f;
	//	f->out = (f->alpha * f->prev_out) + (f->alpha * (input - f->prev_in));
	f->out = f->alpha * (f->prev_out + input - f->prev_in);//code refactored ver
	return f->out / 1.0f;
}

float exp_smoothing(filter *f, float input){
	f->start = HAL_GetTick();
	f->dt = f->start - f->last;
	if(f->dt >= f->sample_time){
		f->out = (f->alpha * (input)) + ((1 - f->alpha) * f->prev_out);
		f->prev_out = f->out;
		f->last = f->start;
	}
	return f->out / 1.0f;
}

float simple_mov_avg(filter *f, float input){
	f->start = HAL_GetTick();
	f->dt = f->start - f->last;
	if(f->dt >= f->sample_time){
		f->out = (input + f->prev_in) / 2.0f;
		f->prev_in = input;
		f->last = f->start;
	}
	return f->out / 1.0f;
}

void alpha_beta_filter(filter *f, float input){
	f->start = HAL_GetTick();
	f->dt = f->start - f->last;
	if(f->dt >= f->sample_time){
		f->xk_prev += f->vk_prev;
		f->rk_now = input - f->xk_prev;
		f->xk_now += (f->A * f->rk_now);
		f->vk_now += (f->rk_now - f->rk_prev) * f->B;

		f->rk_prev = f->rk_now;
		f->xk_prev = f->xk_now;
		f->vk_prev = f->vk_now;
		f->last = f->start;
	}
}

float kalman_get_angle(filter *f, float new_angle, float new_angle_rate){
	f->start = HAL_GetTick();
	f->dt = f->start - f->last;
	if(f->dt >= f->sample_time){
		f->angle += (new_angle_rate - f->bias);

		f->P[0][0] += (f->P_[1][1] - f->P_[0][1] - f->P_[1][0] + f->Q_angle);
		f->P[0][1] -= f->P_[1][1];
		f->P[1][0] -= f->P_[1][1];
		f->P[1][0] += f->Q_bias;

		f->S = f->P[0][0] + f->R_measure;
		f->K[0] = f->P[0][0] / f->S;
		f->K[1] = f->P[1][0] / f->S;

		f->angle += f->K[0] * (new_angle - f->angle);
		f->bias += f->K[1] * (new_angle - f->angle);

		f->P_[0][0] -= f->K[0] * f->P[0][0];
		f->P_[0][1] -= f->K[0] * f->P[1][0];
		f->P_[1][0] -= f->K[1] * f->P[0][0];
		f->P_[1][1] -= f->K[1] * f->P[1][0];
	}
	return f->angle;
}
