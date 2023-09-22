/*
 * motor_control.c
 *
 *  Created on: Jan 26, 2023
 *      Author: udin
 */

#include "Handler.h"
#include "motor_control.h"

/* bldc */
esc bldc_1 = {.htim = &htim12, .channel = TIM_CHANNEL_1, .min_us = 1100, .max_us = 1900};
esc bldc_2 = {.htim = &htim12, .channel = TIM_CHANNEL_2, .min_us = 1100, .max_us = 1900};
/* omni & mechanism */
motor_channel LeftFront = {
		.in1_ = &htim3, .in2_ = &htim3,
		.ch1_ = TIM_CHANNEL_1, .ch2_ = TIM_CHANNEL_4,
		//.en_port = ENA_L_FRONT_GPIO_Port,
		//.en_pin = ENA_L_FRONT_Pin
		.en_port = DRV_ENABLE_L_FRONT_GPIO_Port,
		.en_pin = DRV_ENABLE_L_FRONT_Pin
};
motor_channel LeftBack = {
		.in1_ = &htim1, .in2_ = &htim1,
		.ch1_ = TIM_CHANNEL_1, .ch2_ = TIM_CHANNEL_2,
		//.en_port = ENA_L_BACK_GPIO_Port,
		//.en_pin = ENA_L_BACK_Pin
		.en_port = DRV_ENABLE_L_BACK_GPIO_Port,
		.en_pin = DRV_ENABLE_L_BACK_Pin
};
motor_channel RightFront = {
		.in1_ = &htim3, .in2_ = &htim3,
		.ch1_ = TIM_CHANNEL_2, .ch2_ = TIM_CHANNEL_3,
		//.en_port = ENA_R_FRONT_GPIO_Port,
		//.en_pin = ENA_R_FRONT_Pin
		.en_port = DRV_ENABLE_R_FRONT_GPIO_Port,
		.en_pin = DRV_ENABLE_R_FRONT_Pin
};
motor_channel RightBack = {
		.in1_ = &htim1, .in2_ = &htim1,
		.ch1_ = TIM_CHANNEL_4, .ch2_ = TIM_CHANNEL_3,
		//.en_port = ENA_R_BACK_GPIO_Port,
		//.en_pin = ENA_R_BACK_Pin
		.en_port = DRV_ENABLE_R_BACK_GPIO_Port,
		.en_pin = DRV_ENABLE_R_BACK_Pin
};
motor_channel X_penembak = {
		.in1_ = &htim4, .in2_ = &htim4,
		.ch1_ = TIM_CHANNEL_2, .ch2_ = TIM_CHANNEL_1,
		.en_port = DRV_ENABLE_X_PENEMBAK_GPIO_Port,
		.en_pin = DRV_ENABLE_X_PENEMBAK_Pin
		//.en_port = ENA_X_Penembak_GPIO_Port,
		//.en_pin = ENA_X_Penembak_Pin
};
motor_channel Y_penembak = {
		.in1_ = &htim4, .in2_ = &htim4,
		//.ch1_ = TIM_CHANNEL_3, .ch2_ = TIM_CHANNEL_4,
		//kelinci
		.ch1_ = TIM_CHANNEL_4, .ch2_ = TIM_CHANNEL_3,
		.en_port = DRV_ENABLE_Y_PENEMBAK_GPIO_Port,
		.en_pin = DRV_ENABLE_Y_PENEMBAK_Pin
		//.en_port = DRV_ENABLE_L_FRONT_GPIO_Port,
		//.en_pin = DRV_ENABLE_L_FRONT_Pin
};
motor_channel Chain_lift = {
		.in1_ = &htim10, .in2_ = &htim11,
		.ch1_ = TIM_CHANNEL_1, .ch2_ = TIM_CHANNEL_1,
		.en_port = DRV_ENABLE_GRIPPER_MT_GPIO_Port,
		.en_pin = DRV_ENABLE_GRIPPER_MT_Pin
};

motor_channel Chain_lift2 = {
		.in1_ = &htim13, .in2_ = &htim14,
		.ch1_ = TIM_CHANNEL_1, .ch2_ = TIM_CHANNEL_1,
		.en_port = DRV_ENABLE_RSVD_GPIO_Port,
		.en_pin = DRV_ENABLE_RSVD_Pin
};

void bldc_init(esc *servo){
	HAL_TIM_PWM_Start(servo->htim, servo->channel);

	float min_ccr_ = __HAL_TIM_GET_AUTORELOAD(servo->htim) * 0.05f;
	float max_ccr_ = __HAL_TIM_GET_AUTORELOAD(servo->htim) * 0.1f;

	servo->min_ccr = lin_interp(servo->min_us, 1000.0f, 2000.0f, min_ccr_, max_ccr_);
	servo->max_ccr = lin_interp(servo->max_us, 1000.0f, 2000.0f, min_ccr_, max_ccr_);

	__HAL_TIM_SET_COMPARE(servo->htim, servo->channel, servo->min_ccr);
}

void bldc_drive(esc *servo, uint32_t *duty){
	if(*duty >= servo->max_ccr) *duty = servo->max_ccr;
	else if(*duty <= servo->min_ccr) *duty = servo->min_ccr;
	else __NOP();
	__HAL_TIM_SET_COMPARE(servo->htim, servo->channel, *duty);
}

void bldc_duty(esc *servo, float percent){
	uint32_t u32_duty = (uint32_t)(lin_interp(percent, 0.0f, 100.0f, servo->min_ccr, servo->max_ccr));
	__HAL_TIM_SET_COMPARE(servo->htim, servo->channel, u32_duty);
}

void motor_init(motor_channel *wheel_n){
	HAL_GPIO_WritePin(wheel_n->en_port, wheel_n->en_pin, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(wheel_n->in1_, wheel_n->ch1_);
	HAL_TIM_PWM_Start(wheel_n->in2_, wheel_n->ch2_);

	__HAL_TIM_SET_COMPARE(wheel_n->in1_, wheel_n->ch1_, 0);
	__HAL_TIM_SET_COMPARE(wheel_n->in2_, wheel_n->ch2_, 0);
}

void motor_drive(motor_channel *wheel_n, int_fast16_t rpm){ //rpm = ccr val
	int_fast16_t dir = rpm;
	uint_fast32_t rpm_ = (uint_fast32_t)(abs(rpm));
	if(dir > 0){
		__HAL_TIM_SET_COMPARE(wheel_n->in1_, wheel_n->ch1_, rpm_);
		__HAL_TIM_SET_COMPARE(wheel_n->in2_, wheel_n->ch2_, 0);
	}
	else if(dir < 0){
		__HAL_TIM_SET_COMPARE(wheel_n->in1_, wheel_n->ch1_, 0);
		__HAL_TIM_SET_COMPARE(wheel_n->in2_, wheel_n->ch2_, rpm_);
	}
	else{
		__HAL_TIM_SET_COMPARE(wheel_n->in1_, wheel_n->ch1_, 0);
		__HAL_TIM_SET_COMPARE(wheel_n->in2_, wheel_n->ch2_, 0);
	}
}

void disable_motor(motor_channel *wheel_n){
	HAL_GPIO_WritePin(wheel_n->en_port, wheel_n->en_pin, GPIO_PIN_RESET);

}

void enable_motor(motor_channel *wheel_n){
	HAL_GPIO_WritePin(wheel_n->en_port, wheel_n->en_pin, GPIO_PIN_SET);
}

