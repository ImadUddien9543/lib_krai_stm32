/*
 * math_formula.c
 *
 *  Created on: Jul 14, 2023
 *      Author: udin
 */

#include "PID.h"
#include "math_formula.h"

//proportional on measurment pid dan lpf untuk derivatve
//note: pid semakin ineffective klo sampling time makin kecil, inget2 nyquist frequency
//ngaruh ke aliasing data


void pid(float *setpoint, float *feedback, pid_a *pid){
	pid->start = HAL_GetTick();
	pid->dt = pid->start - pid->last;
	if(pid->dt >= pid->ts){
		pid->error = (*setpoint) - (*feedback);
		pid->d_input = (*feedback) - pid->prev_feedback;

		pid->lpf_new = pid->alpha * pid->d_input + ((1 - pid->alpha) * pid->lpf_last);//lpf

		//		pid->out_sum += (pid->ki * pid->error);	//rectangular
		pid->out_sum += (pid->ki * (pid->error + pid->prev_error) * 0.5f); //trapezoidal
		pid->out_sum -= (pid->kp * pid->d_input); //proportional on measurement

		if(pid->out_sum > pid->out_max) pid->out_sum = pid->out_max;
		else if(pid->out_sum < pid->out_min) pid->out_sum = pid->out_min;

		pid->output += (pid->out_sum - (pid->kd * pid->lpf_new));

		if(pid->output > pid->out_max) pid->output = pid->out_max;
		else if(pid->output < pid->out_min) pid->output = pid->out_min;

		pid->prev_error = pid->error;
		pid->prev_feedback = (*feedback);
		pid->lpf_last = pid->lpf_new;
		pid->last = pid->start;
	}
}

//https://apmonitor.com/pdc/index.php/Main/ProportionalIntegralDerivative
void pid_2(float *setpoint, float *feedback, pid_a *pid){
	pid->start = HAL_GetTick();
	pid->dt = pid->start - pid->last;
	if(pid->dt >= pid->ts){
		pid->error = (*setpoint) - (*feedback);
		pid->d_input = (*feedback) - pid->prev_feedback;

		//		pid->out_sum += (pid->ki * pid->error);	//rectangular
		pid->out_sum += (pid->ki * (pid->error + pid->prev_error) * 0.5f); //trapezoidal
		pid->out_sum -= (pid->kp * pid->d_input); //proportional on measurement

		if(pid->out_sum > pid->out_max) pid->out_sum = pid->out_max;
		else if(pid->out_sum < pid->out_min) pid->out_sum = pid->out_min;

		// out(t) = Kp * e(t) + Ki * ((e(t) + e(t-1)) / 2) - Kd * d(PV)/dt - a * d(out(t))/dt
		pid->output += (pid->out_sum - (pid->kd * pid->d_input));
		pid->d_output = pid->output - pid->prev_output;
		pid->lpf_new = pid->lpf_last + pid->alpha * ((pid->output - pid->prev_output) - pid->lpf_last); //filtered output
		pid->output -= pid->lpf_new;

		if(pid->output > pid->out_max) pid->output = pid->out_max;
		else if(pid->output < pid->out_min) pid->output = pid->out_min;

		pid->prev_error = pid->error;
		pid->prev_output = pid->output;
		pid->prev_feedback = (*feedback);
		pid->lpf_last = pid->lpf_new;
		pid->last = pid->start;
	}
}


//=================================================================================================

void pid_3(float *setpoint, float *feedback, pid_b *pid){
	pid->start = HAL_GetTick();
	pid->dt = pid->start - pid->last; //ke detik
	pid->tau = pid->kd / (pid->kp * pid->N);
	pid->alpha = pid->ts / (2.0f * pid->tau);

	if(pid->dt >= pid->ts){
		pid->pv[2] = pid->pv[1];
		pid->pv[1] = pid->pv[0];
		pid->pv[0] = *feedback - pid->last_fb;

		pid->error[2] = pid->error[1];
		pid->error[1] = pid->error[0];
		pid->error[0] = *setpoint - *feedback;
		//PI
		pid->A[0] = pid->kp * (pid->error[0] - pid->error[1]);
		pid->A[1] += (pid->error[0] + pid->p_error[0]) * 0.5f * pid->ki;
		pid->output += pid->A[0] + pid->A[1];
		//Filtered D
		pid->d[1] = pid->d[0];

		pid->A_d[0] = (pid->pv[0]) * -pid->kd;
		pid->A_d[1] = (pid->pv[1]) * pid->kd * -2.0f;
		pid->A_d[2] = (pid->pv[2]) * pid->kd;

		pid->d[0] = pid->A_d[0] + pid->A_d[1] + pid->A_d[2] - (pid->output - pid->last_out);

		pid->fd[1] = pid->fd[0];
		pid->fd[0] = (pid->alpha / (pid->alpha + 1.0f)) * (pid->d[0] + pid->d[1]) - (pid->alpha / (pid->alpha + 1.0f)) * pid->fd[1];

		pid->output += pid->fd[0];
		pid->output = clamp(pid->output, pid->out_min, pid->out_max);

		pid->last_fb = *feedback;
		pid->last_out = pid->output;
		pid->p_error[0] = pid->error[0];
		pid->p_error[1] = pid->error[1];
		pid->p_error[2] = pid->error[2];

		pid->last = pid->start;
	}
}
