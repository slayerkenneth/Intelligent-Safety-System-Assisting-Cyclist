
#include "a3144.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "math.h"

TIM_HandleTypeDef htim3;

#define A3144_Port GPIOB
#define A3144_PIN GPIO_PIN_13
#define Time_Out_Period 3

float rotSpeed = 0;
int rotCount = 0;

static uint16_t firstDetected;
static uint16_t newDetected;
static float time_diff;

void CountRPM(uint16_t counter)
{
	static uint8_t flip = 0;

	if (flip != HAL_GPIO_ReadPin(A3144_Port, A3144_PIN))
	{
		flip = HAL_GPIO_ReadPin(A3144_Port, A3144_PIN);
		rotCount++;

		if (rotCount == 1){
			firstDetected = counter;
		}

		else {
			newDetected = counter;

			time_diff = abs(newDetected - firstDetected);
			rotSpeed = time_diff / 1000 ;

			rotCount = 0; //reset
		}
	}
}

float GetRotationSpeed(void)
{
//	static float beforeSpeed = 0;
	static int clear = 0;
	clear++;
//
	if (clear == Time_Out_Period) //Time-out: stopped rotation => reset
	{
		clear = 0;
		rotSpeed *= 1.45;
	}

//	if (rotSpeed != 0)
//	{
//		clear = 0;
//
//		beforeSpeed = rotSpeed;
//		rotSpeed = 0;
//	}

	return 30 / rotSpeed;
}

void a3144_Init(void)
{
	HAL_TIM_Base_Start_IT(&htim3);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)   // Interrupt Call
{
	static uint16_t counter_for_1s = 0;


	if (htim == &htim3)
	{
		CountRPM(counter_for_1s);
		counter_for_1s++;
	}
}


