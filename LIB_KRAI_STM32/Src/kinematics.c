/*
 * kinematics.c
 *
 *  Created on: Sep 30, 2023
 *      Author: Imaduddien
 */

#include "kinematics.h"


/*	OMNI 4 roda
 *	Jika yg arg th dalam derajat, ganti fungsi dari cosf() ke cos_()
 *	fungsi inverse untuk alpha = [pi/4, 3pi/4, -3pi/4, -pi/4]
 * */
void inv_k_omni(inverse *i, float vx, float vy, float vz, float th){
	i->rf = vx * M_SQRT1_2 * (cosf(th) - sinf(th)) + vy * M_SQRT1_2 * (cosf(th + sinf(th))) + vz * i->L;
	i->lf = vx * -M_SQRT1_2 * (cosf(th) + sinf(th)) + vy * M_SQRT1_2 * (cosf(th - sinf(th))) + vz * i->L;
	i->lb = vx * -M_SQRT1_2 * (cosf(th) - sinf(th)) + vy * -M_SQRT1_2 * (cosf(th + sinf(th))) + vz * i->L;
	i->rb = vx * M_SQRT1_2 * (cosf(th) + sinf(th)) + vy * -M_SQRT1_2 * (cosf(th - sinf(th))) + vz * i->L;
}

/*	SWERVE 4 roda
 *	Jika yg arg th dalam derajat, ganti fungsi dari atan2f() ke atan()
 * */
void inv_k_swerve_4(inverse *i, float vx, float vy, float vz, float th){
	i->A = vx - (vz * i->L * 0.5);
	i->B = vx + (vz * i->L * 0.5);
	i->C = vy - (vz * i->W * 0.5);
	i->D = vy + (vz * i->W * 0.5);

	i->v1 = sqrtf((i->C * i->C) + (i->B * i->B));
	i->v2 = sqrtf((i->D * i->D) + (i->B * i->B));
	i->v3 = sqrtf((i->D * i->D) + (i->A * i->A));
	i->v4 = sqrtf((i->C * i->C) + (i->A * i->A));

	i->a1 = atan2f(i->C, i->B);
	i->a2 = atan2f(i->D, i->B);
	i->a3 = atan2f(i->D, i->A);
	i->a4 = atan2f(i->C, i->A);
}

/*
 * ODOM 3 roda
 * Jika yg arg th dalam radian, ganti fungsi dari cos_() ke cosf()
 * untuk alpha = [0, pi, -pi/2]
 * */
void fwd_k(odom *o, float q1, float q2, float q3, float th){
	o->x = q1 * 0.5f * (cosf(th) - sinf(th)) + q2 * -0.5f * (cosf(th) - sinf(th));
	o->y = q1 * 0.5f * (cosf(th) + sinf(th)) + q2 * 0.5f * (cosf(th) - sinf(th)) - q3;
	o->z = (q1 * (cosf(th) + sinf(th)) + q2 * (cosf(th) - sinf(th))) * (1/o->l);
}

//https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
//https://www.chiefdelphi.com/t/swerve-drive-direct-and-reverse-kinematics/395803
//https://schreiaj.github.io/swerve_math_demo/

float cos_(float deg){
	return cosf(deg * M_PI/180.0f);
}

float sin_(float deg){
	return sinf(deg * M_PI/180.0f);
}
