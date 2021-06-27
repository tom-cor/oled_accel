/*
 * mpu6050.c
 *
 *  Created on: Jun 27, 2021
 *      Author: tomas
 */

#include "mpu6050.h"

RAW_DATA raw;

void mpu6050_Init (MPU6050 *mpu6050)
{
	mpu6050->accel_x = 0;
	mpu6050->accel_y = 0;
	mpu6050->accel_z = 0;

	mpu6050->temp = 0;

	mpu6050->gyro_x = 0;
	mpu6050->gyro_y = 0;
	mpu6050->gyro_z = 0;
}

void mpu6050_Get_Accel(MPU6050 *mpu6050)
{
	HAL_I2C_Mem_Read(&MPU6050_I2C_PORT, MPU6050_ADDRESS | 0, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);


	raw.accel_x = (Rec_Data[0] << 8 | Rec_Data [1]);
	raw.accel_y = (Rec_Data[2] << 8 | Rec_Data [3]);
	raw.accel_z = (Rec_Data[4] << 8 | Rec_Data [5]);

	mpu6050->accel_x = (float)raw.accel_x / 16384.0;
	mpu6050->accel_y = (float)raw.accel_y / 16384.0;
	mpu6050->accel_z = (float)raw.accel_z / 16384.0;
}
