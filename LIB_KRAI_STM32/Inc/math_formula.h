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

#define ATOL 1e-08
#define RTOL 1e-05


#ifdef __cplusplus
extern "C" {
#endif

#define ARRAY_LEN(array) (sizeof(array) / sizeof(array[0]))

typedef float (*order)(float);

typedef struct Poly_Regression {
    float c4, c3, c2, c1, c0;
} pr;

typedef struct SmoothStep {
    float in, min, max;
} SStep;

extern pr m[6];
extern SStep a;

float polyregress(pr *pr, const float input);

float smooth_step(SStep *s, float in, order ord);
float sstep_1th(float x);
float sstep_2th(float x);
float sstep_3th(float x);

float clamp(float val, float min, float max);
float lin_interp(float x, float in_min, float in_max, float out_min, float out_max);

/*floating point error (inexact)
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

