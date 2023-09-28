/*
 * polynomial_regression.h
 *
 *  Created on: Jul 14, 2023
 *      Author: udin
 */

#ifndef INC_POLYNOMIAL_REGRESSION_H_
#define INC_POLYNOMIAL_REGRESSION_H_

#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "math.h"
#include "stdlib.h"
#include "stdbool.h"

#define TOLERANCE_F 1e-08

#define ARRAY_LEN(array) (sizeof(array) / sizeof(array[0]))

#ifdef __cplusplus
extern "C" {
#endif

typedef float (*order)(float);

typedef enum{
	ORD1 = 0x01U,
	ORD2 = 0x02U,
	ORD3 = 0x03U,
}SSTEP_ORD;

typedef struct _m_struct_ {
	//polynomial regression member
	float c0, c1, c2, c3, c4;
	float (*get_poly)(struct _m_struct_*, float);
	//smooth_step member
	float in, ss_min, ss_max;
	float (*get_calc)(struct _m_struct_*, float);
	order ord;
    //inverse kinematics member
    float rf, lf, rb, lb;
    float vel_coeff[3]; //vector (vx, vy , vz) buat adjust saja
} m_struct;

m_struct *init_inverse(float vel_coeff_x, float vel_coeff_y, float vel_coeff_z);
m_struct *init_polyregress(float c0, float c1, float c2, float c3, float c4);
m_struct *init_smooth_step(SSTEP_ORD ord, float min, float max);

float clamp(float val, float min, float max);
float lin_interp(float x, float in_min, float in_max, float out_min, float out_max);
float round_(float val);

/*
 *	INVERSE KINEMATICS
 * */
void calc_inv_k(m_struct *self, float vel_x, float vel_y, float vel_z);

/*floating point error handling(inexact)
	https://floating-point-gui.de/
	https://stackoverflow.com/questions/70780366/why-should-you-ever-use-a-float-instead-of-a-double
	https://stackoverflow.com/questions/4584637/double-or-float-which-is-faster
	https://docs.oracle.com/cd/E19957-01/806-3568/ncg_goldberg.html
	https://fastbitlab.com/microcontroller-embedded-c-programming-lecture-64-working-with-float-and-double-variables/
*/
float close_to_zero(float value);

#ifdef __cplusplus
}
#endif

#endif /* INC_POLYNOMIAL_REGRESSION_H_ */
