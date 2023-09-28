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
    float out, prev_out, prev_in;
    float alpha, RC;
    float dt_, ts_;	//float dt dalam s
    uint32_t start, last, dt, sample_time;
    //alpha-beta
    float A, B, xk_prev, vk_prev, xk_now, vk_now, rk_prev, rk_now;
    //kalman
    float Q_angle, Q_bias, R_measure, angle, bias, P[2][2], P_[2][2], S, K[2];
} filter;

float low_pass(filter *f, float input);
float hi_pass(filter *f, float input);
float exp_smoothing(filter *f, float input);
float simple_mov_avg(filter *f, float input);
void alpha_beta_filter(filter *f, float input);
float kalman_get_angle(filter *f, float new_angle, float new_angle_rate);

#ifdef __cplusplus
}
#endif

#endif /* INC_FILTERS_H_ */
