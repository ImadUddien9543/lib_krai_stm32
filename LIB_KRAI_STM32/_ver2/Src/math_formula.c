/*
 * math_formula.c
 *
 *  Created on: Jul 14, 2023
 *      Author: udin
 */

#include "math_formula.h"

m_struct m[6] = { //motor 1
		{.c0 = -5.0691673081360022e-012, .c1 = 1.5957142857153541e+000, .c2 = -3.8571428571550128e-003
				,.c3 = 9.8571428571836824e-006, .c4 = -8.5714285714700861e-009},	//rf

				{.c0 = -9.7764019102442035e-012, .c1 = 1.7242857142877093e+000, .c2 = -5.0011904762130811e-003
						,.c3 = 1.3892857142932903e-005, .c4 = -1.3095238095315033e-008},	//lf

						{.c0 = -7.1500583231909332e-012, .c1 = 1.5760317460332156e+000, .c2 = -4.3658730158896951e-003
								,.c3 = 1.2182539682595640e-005,	.c4 = -1.1269841269898140e-008},	//lb

								{.c0 = -7.3461237093397358e-012, .c1 = 1.7126190476206040e+000, .c2 = -4.7928571428748510e-003
										,.c3 = 1.2809523809583281e-005, .c4 = -1.1428571428631899e-008},	//rb

										{.c0 = -9.1706642280087181e-012, .c1 = 1.8608730158749087e+000, .c2 = -5.4281746031960964e-003
												,.c3 = 1.4519841269913400e-005, .c4 = -1.3253968254041571e-008},	//ll

												{.c0 = -7.3324679661368464e-012, .c1 = 1.6055555555570582e+000, .c2 = -4.0111111111281548e-003
														,.c3 = 1.0444444444501601e-005, .c4 = -8.8888888889469583e-009}		//rr
};

/*
 * pemanggilan:
 * di int main() atau misal rtos di fungsi task nya (BUKAN DI LOOP e.g. while(1) atau for(;;))
 *
 *		m_struct *obj = init():
 *  BARU DI LOOP NYA:
 *		motor_drive(&L_Back, obj->lb);
 * */

static float polyregress(m_struct *pr, float input);
static float sstep_1th(float x);
static float sstep_2th(float x);
static float sstep_3th(float x);
static float smooth_step(m_struct *s, float in);

m_struct *init_inverse(float vel_coeff_x, float vel_coeff_y, float vel_coeff_z){
	m_struct *obj;
	obj = malloc(sizeof(*obj));
	obj->vel_coeff[0] = vel_coeff_x;
	obj->vel_coeff[1] = vel_coeff_y;
	obj->vel_coeff[2] = vel_coeff_z;
	return (m_struct*)obj;
}

m_struct *init_polyregress(float c0, float c1, float c2, float c3, float c4){
	m_struct *obj;
	obj = malloc(sizeof(*obj));
	obj->c0 = c0;
	obj->c1 = c1;
	obj->c2 = c2;
	obj->c3 = c3;
	obj->c4 = c4;
	obj->get_poly = polyregress;
	return (m_struct*)obj;
}

m_struct *init_smooth_step(SSTEP_ORD ord, float min, float max){
	m_struct *obj;
	obj = malloc(sizeof(*obj));
	obj->ss_min = min;
	obj->ss_max = max;

	if(ord == ORD2) obj->ord = sstep_2th;
	else if(ord == ORD3) obj->ord = sstep_3th;
	else obj->ord = sstep_1th;

	obj->get_calc = smooth_step;
	return (m_struct*)obj;
}

//result dalam hanya nilai positif (fabs)
static float polyregress(m_struct *pr, float input){
	float out_ =
			pr->c4 * fabs(input) * fabs(input) * fabs(input) * fabs(input) +
			pr->c3 * fabs(input) * fabs(input) * fabs(input) +
			pr->c2 * fabs(input) * fabs(input) +
			pr->c1 * fabs(input) +
			pr->c0 * 1.0f;

	return close_to_zero(out_);
}

// invoke: float hasil = smooth_step(&a, t, sstep_3th);
static float smooth_step(m_struct *s, float in){
	s->in = lin_interp(in, s->ss_min, s->ss_max, 0.0f, 1.0f);
	float x_ = s->ord(s->in) * 1.0f;
	float y_ = lin_interp(x_, 0.0f, 1.0f, s->ss_min, s->ss_max);
	return close_to_zero(y_);
}

static float sstep_1th(float x){
	return x * x * (3.0f - 2.0f * x) / 1.0f;
}
static float sstep_2th(float x){
	return x * x * x * (x * (6.0f * x - 15.0f) + 10.0f) / 1.0f;
}
static float sstep_3th(float x){
	return (-20.0f * pow(x, 7) + 70.0f * pow(x, 6) - 84.0f * pow(x, 5) + 35.0f * pow(x, 4));
}



//inverse yg sudah sangat sangat-sangat disederhanakn khusus joystick manual (DI CHECK DULU OUTPUT JOYSTICK YG TERBACA BAGAIMANA)
//taro di loop
void calc_inv_k(m_struct *self, float vel_x, float vel_y, float vel_z){
	self->rf = (-1.0f * vel_x * self->vel_coeff[0]) + (1.0f * vel_y * self->vel_coeff[1]) + (vel_z * self->vel_coeff[2]);
	self->lf = (-1.0f * vel_x * self->vel_coeff[0]) + (-1.0f * vel_y * self->vel_coeff[1]) + (vel_z * self->vel_coeff[2]);
	self->lb = (1.0f * vel_x * self->vel_coeff[0]) + (-1.0f * vel_y * self->vel_coeff[1]) + (vel_z * self->vel_coeff[2]);
	self->rb = (1.0f * vel_x * self->vel_coeff[0]) + (1.0f * vel_y * self->vel_coeff[1]) + (vel_z * self->vel_coeff[2]);
}

float lin_interp(float x, float in_min, float in_max, float out_min, float out_max){
	float clamp_x = clamp(x, in_min, in_max);
	float out_interp = (clamp_x - in_min) * (out_max - out_min) / (in_max - in_min) + (out_min);
	float clamp_out_ = clamp(out_interp, out_min, out_max);
	return close_to_zero(clamp_out_);
}

float close_to_zero(float value){
	if(fabs(value) <= TOLERANCE_F) return 0.0f;
	else return value * 1.0f;
}

float clamp(float val, float min, float max){
	if(val > max) return max * 1.0f;
	else if(val < min) return min * 1.0f;
	else return val * 1.0f;
}

float round_(float val){
	if(close_to_zero(val) == 0.0f) return 0.0f;
	else return (val < 0.0f) ? (floorf(val) + 0.5f) : (ceilf(val) + 0.5f);

}
