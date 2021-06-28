/*
 * angles.c
 *
 *  Created on: Jun 28, 2021
 *      Author: tomas
 */

#include "angles.h"


void angles_update(MPU6050 *mpu6050, ANGLES *angle)
{
	FIRFilter_Update(&az_filter, mpu6050->accel_z);

	angle->yx = -1*(atan2(mpu6050->accel_y,mpu6050->accel_x)*180)/PI;
	angle->xz = (atan2(mpu6050->accel_x,mpu6050->accel_z)*180)/PI;
	angle->yz = -1*(atan2(mpu6050->accel_y,mpu6050->accel_z)*180)/PI;

	angle->yx = FIRFilter_Update(&angle_yx_filter, angle->yx);
	angle->xz = FIRFilter_Update(&angle_xz_filter, angle->xz);
	angle->yz = FIRFilter_Update(&angle_yz_filter, angle->yz);

	return;
}
