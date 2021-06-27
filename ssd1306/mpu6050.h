/*
 * mpu6050.h
 *
 *  Created on: Jun 27, 2021
 *      Author: tomas
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include <stddef.h>
#include <_ansi.h>

_BEGIN_STD_C

#include "mpu6050_conf.h"

#include "stm32f1xx_hal.h"

/* vvv I2C config vvv */

#ifndef MPU6050_I2C_PORT
#define MPU6050_I2C_PORT        hi2c1
#endif

#ifndef MPU6050_ADDRESS
#define MPU6050_ADDRESS        (0x68 << 1)
#endif

/* ^^^ I2C config ^^^ */

extern I2C_HandleTypeDef MPU6050_I2C_PORT;

#define PWRMNGT1_REG  		0x6B
#define ACCEL_XOUT_H_REG	0x3B
#define WHO_AM_I  			0x75

typedef struct{

	float accel_x;
	float accel_y;
	float accel_z;
	float temp;
	float gyro_x;
	float gyro_y;
	float gyro_z;

} MPU6050;

typedef struct{

	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t temp;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;

} RAW_DATA;

uint8_t Rec_Data[14];


void mpu6050_Init (MPU6050 *mpu6050);
void mpu6050_Get_Accel(MPU6050 *mpu6050);

_END_STD_C

#endif /* MPU6050_H_ */
