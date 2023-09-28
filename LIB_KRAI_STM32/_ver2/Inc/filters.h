/*
 * filters.h
 *
 *  Created on: Sep 10, 2023
 *      Author: Imaduddien
 */

#ifndef INC_FILTERS_H_
#define INC_FILTERS_H_

#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "math.h"
#include "stdlib.h"
#include "stdbool.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef struct Filter_ {
    //low & high pass filter, double exponential smoothing
	// 0 <= alpha <= 1
	// 0 <= a <= 1, 		0 << b << 1
	float out, prev_out, prev_in;
    float alpha;
    uint32_t start, last, dt, sample_time;
    void (*get_val)(struct Filter_*, float);
    float a, b;
    float st[2], bt[2];
    //alpha-beta
    float A, B, xk[2], vk[2], rk[2];
    void (*get_alpha_beta)(struct Filter_*, float);
    //kalman
    float Q_angle, Q_bias, R_measure, angle, bias, P[2][2], P_[2][2], S, K[2];
    void (*get_kalman)(struct Filter_*, float, float);


} filter;

filter *lpf_init(float alpha, uint32_t ts);
filter *hpf_init(float alpha, uint32_t ts);
filter *double_exp_init(float a, float b, uint32_t ts);
filter *kalman_init(float Q_angle, float Q_bias, float R_measure, uint32_t ts);
filter *alpha_beta_init(float A, float B, uint32_t ts);


#ifdef __cplusplus
}
#endif

#endif /* INC_FILTERS_H_ */
