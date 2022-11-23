/*
 * ws2812.h
 *
 *  Created on: Nov 15, 2022
 *      Author: kennethlok
 */

#ifndef INC_WS2812_H_
#define INC_WS2812_H_

#define MAX_LED 14
#define USE_BRIGHTNESS 1

#include "main.h"
#include "math.h"


uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];  // for brightness
uint16_t pwmData[(24*MAX_LED)+50];
uint16_t effStep;


int datasentflag;

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void Set_LED (int LEDnum, int Red, int Green, int Blue);
void Set_Brightness (int brightness);  // 0-45
void WS2812_Send (void);

void Reset_LED (void);
uint8_t rainbow_effect_left();
uint8_t rainbow_effect_right();

#endif /* INC_WS2812_H_ */
