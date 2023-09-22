/*
 * filters.c
 *
 *  Created on: Sep 10, 2023
 *      Author: Imaduddien
 */

#include "filters.h"


float low_pass(filter *f, float input){
	f->start = HAL_GetTick();
	f->sample_time = f->start - f->last; //second
	if(f->dt >= f->sample_time){
		f->out = f->prev_out + (f->alpha * (input - f->prev_out));
		f->prev_out = f->out;
		f->last = f->start;
	}
	return f->out / 1.0f;
}

float hi_pass(filter *f, float input){
	f->start = HAL_GetTick();
	f->sample_time = f->start - f->last; //second
	if(f->dt >= f->sample_time){
		f->out = f->alpha * (f->prev_out + input - f->prev_in);
		f->prev_out = f->out;
		f->prev_in = input;
		f->last = f->start;
	}
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

a_b_val *alpha_beta_filter(filter *f, float input){
	f->start = HAL_GetTick();
	f->dt = f->start - f->last;
	if(f->dt >= f->sample_time){
		f->xk[0] += f->vk[0];
		f->rk[0] = input - f->xk[0];
		f->xk[1] += (f->A * f->rk[0]);
		f->vk[1] += (f->rk[1] - f->rk[0]) * f->B;
		f->result->xk = f->xk[1];
		f->result->vk = f->vk[1];

		f->rk[0] = f->rk[1];
		f->xk[0] = f->xk[1];
		f->vk[0] = f->vk[1];
		f->last = f->start;
	}
	return f->result;
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
