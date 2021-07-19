/*
 * hc-sr04.c
 *
 *  Created on: Jul 18, 2021
 *      Author: tomas
 */

#include "hc-sr04.h"

DATA data;

void delay (uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER (&htim1) < time);
}

void HCSR04_Read (void)
{
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}

uint16_t HCSR04_Read_ISR(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		data.captured_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
	}

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		data.captured_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);  // read second value
		__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

		if (data.captured_2 > data.captured_1)
		{
			data.difference = data.captured_2-data.captured_1;
		}

		else if (data.captured_1 > data.captured_2)
		{
			data.difference = (0xffff - data.captured_1) + data.captured_2;
		}
		data.distance = data.difference * .034/2;
		__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
	}

	return data.distance;
}
