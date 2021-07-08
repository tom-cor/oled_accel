/*
 * gui_multitool.c
 *
 *  Created on: Jul 8, 2021
 *      Author: tomas
 */

#include "gui_multitool.h"

void gui_Init(void)
{

}

void gui_WelcomeScreen(void)
{

	ssd1306_Fill(Black);
	ssd1306_SetCursor(1, 3);
	ssd1306_WriteString("Sistemas Embebidos", Font_7x10, White);
	ssd1306_SetCursor(1, 19);
	ssd1306_WriteString("Proyecto final:", Font_7x10, White);
	ssd1306_SetCursor(1, 35);
	ssd1306_WriteString("NIVEL DE BURBUJA", Font_7x10, White);
	ssd1306_SetCursor(1, 51);
	ssd1306_WriteString("Tomas Cornaglia", Font_7x10, White);
	ssd1306_UpdateScreen();

	return;

}

void gui_Bubble_1d(float angle)
{

	static uint8_t radius = 9;
	static int16_t x0;
	const uint8_t y0 = 52;
	const uint8_t x1 = 10;
	const uint8_t x2 = 120;

	//angle = -170;

	//x0 = ((angle + 180)/360.0)*(x2 - x1 - 2*(radius + 1)) + (x1 + radius +1);
	//x0 = (angle/60.0)*45.0 + 65;
	x0 = (angle/60.0)*(x2 - (radius+1) - (x2 + x1)/2) + (x2 + x1)/2;

	if( (x0 - (radius +1)) <= x1 )
		x0 = x1 + (radius + 1);
										//	SON NECESARIOS ESTOS LÍMITES?
	if( (x0 + (radius +1)) >= x2 )
		x0 = x2 - (radius + 1);

	sprintf(MSG0, "%+4.1f", angle);

	ssd1306_Fill(Black);
	ssd1306_DrawRectangle(x1, y0 - (radius + 1), x2, y0 + (radius +1), White);
	ssd1306_DrawCircle(x0, y0, radius, White);
	ssd1306_DrawCircle(x0+3, y0-3, 2, White);
	ssd1306_SetCursor(26, 8);
	ssd1306_WriteString(MSG0, Font_16x26, White);
	ssd1306_UpdateScreen();

	return;

}

void gui_Bubble_2d(float angle_xz, float angle_yz)
{

	sprintf(MSG0, "X: %+.1f", angle_xz);
	sprintf(MSG1, "Y: %+.1f", angle_yz);

	uint8_t x0 = 95;
	uint8_t y0 = 32;

//	x0 += angle_xz;
//	y0 += angle_yz;

	//	CONVERSION A COORDENADAS POLARES

	float radius;
	float theta;

	//radius = sqrt(pow(angle_yz, 2) + pow(angle_xz, 2));
	radius = sqrt(angle_yz*angle_yz + angle_xz*angle_xz);

	if(radius > 26)
		radius = 26;

	theta = atan2(angle_xz, angle_yz);

	x0 += radius * sin(theta);
	y0 -= radius * cos(theta);

	//	FIN CONVERSION A COORDENADAS POLARES

	ssd1306_Fill(Black);
	ssd1306_DrawCircle(95, 32, 31, White);
	//ssd1306_DrawRectangle(63, 1, 126, 63, COLOR);	//	Descomentar en caso de no usar coordenadas polares
	ssd1306_Line(64, 32, 126, 32, White);
	ssd1306_Line(95, 1, 95, 63, White);
	ssd1306_DrawRectangle(89, 26, 101, 38, White);
	ssd1306_DrawCircle(x0, y0, 5, White);
	ssd1306_DrawCircle(x0+2, y0-2, 1, White);
	ssd1306_SetCursor(1, 12);
	ssd1306_WriteString(MSG0, Font_7x10, White);
	ssd1306_SetCursor(1, 42);
	ssd1306_WriteString(MSG1, Font_7x10, White);
	ssd1306_UpdateScreen();

	return;

}

//void bubbleLevel_ArtifHorizon(int16_t angle)
//{
//	static uint16_t x1, y1, x2, y2;
//	static uint16_t radius = 31;
//	static uint16_t x0 = 96;
//	static uint16_t y0 = 32;
//	static char  MSG0[4] = "";
//	static char MSG1[2] = " +";
//
//	x2 = radius*cos((angle*PI)/180) + x0;
//	y2 = radius*sin((angle*PI)/180) + y0;
//	x1 = 2*x0 - x2;
//	y1 = 2*y0 - y2;
//
//	if(angle < 0)
//	{
//		MSG1[1] = '-';
//		angle *= (-1);
//	}
//	else
//		MSG1[1] = '+';
//
//	sprintf(MSG0, "%03d", angle);
//
//	ssd1306_Fill(Black);
//	//ssd1306_SetCursor(5, 19);
//	ssd1306_SetCursor(5, 2);
//	ssd1306_WriteString(MSG1, Font_16x26, White);
//	ssd1306_SetCursor(5, 34);
//	ssd1306_WriteString(MSG0, Font_16x26, White);
//	ssd1306_DrawCircle(x0, y0, radius, White);
//	ssd1306_Line(x1, y2, x2, y1, White);
//	ssd1306_UpdateScreen();
//
//	return;
//}
