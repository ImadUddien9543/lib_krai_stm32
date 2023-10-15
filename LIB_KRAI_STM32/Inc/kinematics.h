/*
 * kinematics.h
 *
 *  Created on: Sep 29, 2023
 *      Author: Imaduddien
 */

#ifndef INC_KINEMATICS_H_
#define INC_KINEMATICS_H_

#include "math.h"

//		cari penyederhanaan matrix di kinematics_solver.py terlebih dahulu
//		untuk meringankan komputasi di stm32
typedef struct base_control{
	float W, L; //meter
	//4 roda omni
	float rf, lf, lb, rb;
	//4 roda swerve
	float A, B, C, D;
	float v1, v2, v3, v4; //wheel speed linear m/s
	float a1, a2, a3, a4; //wheel angle degree
	float vel_x, vel_y, vel_z;
}inverse;

typedef struct odometry_3_wheel{
	float l, x, y, z;
}odom;

void inv_k_omni(inverse *i, float vx, float vy, float vz, float th);
void inv_k_swerve_4(inverse *i, float vx, float vy, float vz, float th);
void fwd_k(odom *o, float q1, float q2, float q3, float th);

float cos_(float deg);
float sin_(float deg);

#endif /* INC_KINEMATICS_H_ */
