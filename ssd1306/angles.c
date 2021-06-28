/*
 * angles.c
 *
 *  Created on: Jun 28, 2021
 *      Author: tomas
 */

#include "angles.h"


void angles_update(MPU6050 *mpu6050, ANGLES *angle)
{
	angle->accel_x = mpu6050->accel_x;
	angle->accel_y = mpu6050->accel_y;
	angle->accel_z = mpu6050->accel_z;

	angle->accel_x = FIRFilter_Update(&ax_filter, angle->accel_x);
	angle->accel_y = FIRFilter_Update(&ay_filter, angle->accel_y);
	angle->accel_z = FIRFilter_Update(&az_filter, angle->accel_z);

	angle->yx = -1*(atan2(angle->accel_y,angle->accel_x)*180)/PI;
	angle->xz = (atan2(angle->accel_x,angle->accel_z)*180)/PI;
	angle->yz = -1*(atan2(angle->accel_y,angle->accel_z)*180)/PI;

	return;
}
