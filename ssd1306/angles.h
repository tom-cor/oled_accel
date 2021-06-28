/*
 * angles.h
 *
 *  Created on: Jun 28, 2021
 *      Author: tomas
 */

#ifndef ANGLES_H_
#define ANGLES_H_

#include <math.h>
#include "mpu6050.h"
#include "FIRFilter.h"

#define PI 3.14159265358

typedef struct
{
	float yx;
	float xz;
	float yz;
	float accel_x;
	float accel_y;
	float accel_z;
} ANGLES;

FIRFilter ax_filter;
FIRFilter ay_filter;
FIRFilter az_filter;

FIRFilter angle_yx_filter;
FIRFilter angle_xz_filter;
FIRFilter angle_yz_filter;

void angles_update(MPU6050 *mpu6050, ANGLES *angle);

#endif /* ANGLES_H_ */
