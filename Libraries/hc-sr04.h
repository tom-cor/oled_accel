/*
 * hc-sr04.h
 *
 *  Created on: Jul 18, 2021
 *      Author: tomas
 */

#ifndef HC_SR04_H_
#define HC_SR04_H_

#include "stm32f1xx_hal.h"
#include "main.h"

#define TRIG_PIN CaptureTrigger_Pin
#define TRIG_PORT CaptureTrigger_GPIO_Port

TIM_HandleTypeDef htim1;

typedef struct{

	uint32_t captured_1;
	uint32_t captured_2;
	uint32_t difference;
	uint16_t distance;

} DATA;

void delay (uint16_t time);
void HCSR04_Read (void);
uint16_t HCSR04_Read_ISR(TIM_HandleTypeDef *htim);


#endif /* HC_SR04_H_ */
