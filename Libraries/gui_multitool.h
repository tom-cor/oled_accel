/*
 * gui_multitool.h
 *
 *  Created on: Jul 8, 2021
 *      Author: tomas
 */

#ifndef GUI_MULTITOOL_H_
#define GUI_MULTITOOL_H_

#include <stdio.h>
#include <math.h>
#include "mpu6050.h"
#include "ssd1306.h"

char MSG0[10];

void gui_Init (void);
void gui_WelcomeScreen (void);
void gui_Bubble_1d (float angle, float temp);
void gui_Bubble_2d (float angle_xz, float angle_yz, float temp);
//void bubbleLevel_ArtifHorizon(int16_t angle);


#endif /* GUI_MULTITOOL_H_ */
