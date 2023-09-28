/*
 * polynomial_regression.h
 *
 *  Created on: Jul 14, 2023
 *      Author: udin
 */

#ifndef INC_PIB_LIB
#define INC_PID_LIB

#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "math.h"
#include "stdlib.h"
#include "stdbool.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _PID_A_{
	uint32_t start, ts, dt, last;
	float error, d_input, d_output, prev_feedback, prev_error;
	float alpha, lpf_new, lpf_last;
	float out_sum, output, prev_output;
	float kp, ki, kd;
	float out_min, out_max;
} pid_a;

typedef struct _PID_B_{
	uint32_t start, ts, dt, last;
	float error[3], p_error[3], A[2], A_d[3], d[2], fd[2], pv[3];
	float tau, alpha, N;
	float output, last_fb, last_out;
	float kp, ki, kd;
	float out_min, out_max;
} pid_b;

extern pid_a m1, m2, m3, m4, m5, m6;

/*
 * sesuaikan time sampling agar tidak terjadi aliasing
 * https://av8rdas.wordpress.com/2023/03/30/pid-loop-tuning-and-aliasing/
 * */

//pid_a
void pid(float *setpoint, float *feedback, pid_a *pid);
void pid_2(float *setpoint, float *feedback, pid_a *pid);

//pid_c
//https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Pseudocode
void pid_3(float *setpoint, float *feedback, pid_b *pid);



#ifdef __cplusplus
}
#endif

#endif /* INC_POLYNOMIAL_REGRESSION_H_ */
