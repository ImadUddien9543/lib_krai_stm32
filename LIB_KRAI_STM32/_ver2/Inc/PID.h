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

typedef enum {
	TYPE_1 = 0x01U,
	TYPE_2 = 0x02U
}PID_TYPE;

typedef struct _PID_A_{
	uint32_t start, ts, dt, last;
	float error, d_input, d_output, prev_feedback, prev_error;
	float alpha, lpf_new, lpf_last;
	float out_sum, output, prev_output;
	float kp, ki, kd;
	float out_min, out_max;
	void(*get_pid)(float*, float*, struct _PID_A_*);
} pid_a;

typedef struct _PID_B_{
	uint32_t start, ts, dt, last;
	float error[3], p_error[3], A[2], A_d[3], d[2], fd[2], pv[3];
	float tau, alpha, N;
	float output, last_fb, last_out;
	float kp, ki, kd;
	float out_min, out_max;
	void(*get_pid)(float*, float*, struct _PID_B_*);
} pid_b;

extern pid_a m1, m2, m3, m4, m5, m6;

pid_a *init_pid_a(float kp, float ki, float kd, float alpha, PID_TYPE type, uint32_t ts);
pid_b *init_pid_b(float kp, float ki, float kd, float alpha, float tau, float N, uint32_t ts);


#ifdef __cplusplus
}
#endif

#endif 
