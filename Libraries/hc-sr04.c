/*
 * hc-sr04.c
 *
 *  Created on: Jul 18, 2021
 *      Author: tomas
 */

//#include "hc-sr04.h"
//
//
//
//uint32_t captured_1 = 0;
//uint32_t captured_2 = 0;
//uint32_t difference = 0;
//uint16_t is_first_captured = 0;  // is the first value captured ?
//uint16_t distance  = 0;
//
//void HCSR04_Read_ISR(TIM_HandleTypeDef *htim)
//{
//	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
//	{
//		captured_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
//	}
//
//	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
//	{
//		captured_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);  // read second value
//		__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
//
//		if (captured_2 > captured_1)
//		{
//			difference = captured_2-captured_1;
//		}
//
//		else if (captured_1 > captured_2)
//		{
//			difference = (0xffff - captured_1) + captured_2;
//		}
//		distance = difference * .034/2;
//		__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
//	}
//
//	return;
//}
