/*
 * filters.c
 *
 *  Created on: Sep 10, 2023
 *      Author: Imaduddien
 */

#include "filters.h"

static void low_pass(filter *f, float input);
static void hi_pass(filter *f, float input);
static void double_exp_smoothing(filter *f, float input);
static void alpha_beta_filter(filter *f, float input);
static void kalman_get_angle(filter *f, float new_angle, float new_angle_rate);

filter *low_pass_init(float alpha, uint32_t ts){
	filter *obj;
	obj = (filter*)malloc(sizeof(obj));
	obj->alpha = alpha;
	obj->sample_time = ts;
	obj->start = obj->last = obj->dt = 0U;
	obj->out = obj->prev_in = obj->prev_out = 0.0f;
	obj->get_val = low_pass;
	return obj;
}

filter *high_pass_init(float alpha, uint32_t ts){
	filter *obj;
	obj = (filter*)malloc(sizeof(obj));
	obj->alpha = alpha;
	obj->sample_time = ts;
	obj->start = obj->last = obj->dt = 0U;
	obj->out = obj->prev_in = obj->prev_out = 0.0f;
	obj->get_val = hi_pass;
	return obj;
}

filter *kalman_init(float Q_angle, float Q_bias, float R_measure, uint32_t ts){
	filter *obj;
	obj = (filter*)malloc(sizeof(obj));
	obj->sample_time = ts;
	obj->Q_angle = Q_angle;
	obj->Q_bias = Q_bias;
	obj->R_measure = R_measure;
	obj->start = obj->last = obj->dt = 0U;
	obj->out = obj->prev_in = obj->prev_out = 0.0f;
	obj->P[0][0] = obj->P[0][1] = obj->P[1][0] = obj->P[1][1] = 0.0f;
	obj->K[0] = obj->K[1] = 0.0f;
	obj->angle = obj->bias = obj->S = 0.0f;
	obj->get_kalman = kalman_get_angle;
	return obj;
}

filter *alpha_beta_init(float A, float B, uint32_t ts){
	filter *obj;
	obj = (filter*)malloc(sizeof(obj));
	obj->sample_time = ts;
	obj->A = A;
	obj->B = B;
	obj->start = obj->last = obj->dt = 0U;
	obj->out = obj->prev_in = obj->prev_out = 0.0f;
	obj->xk[0] = obj->xk[1] = 0.0f;
	obj->vk[0] = obj->vk[1] = 0.0f;
	obj->rk[0] = obj->rk[1] = 0.0f;
	obj->get_alpha_beta = alpha_beta_filter;
	return obj;
}

filter *double_exp_init(float a, float b, uint32_t ts){
	filter *obj;
	obj = (filter*)malloc(sizeof(obj));
	obj->sample_time = ts;
	obj->a = a;
	obj->b = b;
	obj->start = obj->last = obj->dt = 0U;
	obj->st[0] = obj->st[1] = 0.0f;
	obj->bt[0] = obj->bt[1] = 0.0f;
	obj->get_val = double_exp_smoothing;
	return obj;
}

static void low_pass(filter *f, float input){
	f->start = HAL_GetTick();
	f->dt = f->start - f->last; //second
	if(f->dt >= f->sample_time){
		f->out = f->prev_out + (f->alpha * (input - f->prev_out));
		f->prev_out = f->out;
		f->last = f->start;
	}
}

static void hi_pass(filter *f, float input){
	f->start = HAL_GetTick();
	f->dt = f->start - f->last; //second
	if(f->dt >= f->sample_time){
		f->out = f->alpha * (f->prev_out + input - f->prev_in);
		f->prev_out = f->out;
		f->prev_in = input;
		f->last = f->start;
	}
}

static void double_exp_smoothing(filter *f, float input){
	f->start = HAL_GetTick();
	f->dt = f->start - f->last;
	if(f->dt >= f->sample_time){
		f->st[1] = f->a * input + (1 - f->a) * (f->st[0] + f->bt[0]);
		f->bt[1] = f->b * (f->st[1] - f->st[0]) + (1 - f->b) * f->bt[0];
		f->st[0] = f->st[1];
		f->bt[0] = f->bt[1];
		f->last = f->start;
	}
}

static void alpha_beta_filter(filter *f, float input){
	f->start = HAL_GetTick();
	f->dt = f->start - f->last;
	if(f->dt >= f->sample_time){
		f->xk[0] += f->vk[0];
		f->rk[0] = input - f->xk[0];
		f->xk[1] += (f->A * f->rk[0]);
		f->vk[1] += (f->rk[1] - f->rk[0]) * f->B;

		f->rk[0] = f->rk[1];
		f->xk[0] = f->xk[1];
		f->vk[0] = f->vk[1];
		f->last = f->start;
	}
}

static void kalman_get_angle(filter *f, float new_angle, float new_angle_rate){
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
}
