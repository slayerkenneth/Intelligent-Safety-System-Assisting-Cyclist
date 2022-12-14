/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcdtp.h"
#include "xpt2046.h"

#include "math.h"
#include <stdio.h>
#include "stdbool.h"

#include "MPU6050.h"
#include "a3144.h"
#include "vl53l1_api.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
//#define EXPANDER_1_ADDR 0x84 // 0x42 << 1
//#define EXPANDER_2_ADDR 0x86 // 0x43 << 1HAL_I2C_Master_Transmit
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// LED Parameters:
#define MAX_LED 14
#define USE_BRIGHTNESS 0


uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];  // for brightness

int datasentflag=0;


// :ED Function callbacks

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_4);
	datasentflag=1;
}

void Set_LED (int LEDnum, int Red, int Green, int Blue)
{
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
}

#define PI 3.14159265

void Set_Brightness (int brightness)  // 0-45
{
#if USE_BRIGHTNESS

	if (brightness > 45) brightness = 45;
	for (int i=0; i<MAX_LED; i++)
	{
		LED_Mod[i][0] = LED_Data[i][0];
		for (int j=1; j<4; j++)
		{
			float angle = 90-brightness;  // in degrees
			angle = angle*PI / 180;  // in rad
			LED_Mod[i][j] = (LED_Data[i][j])/(tan(angle));
		}
	}

#endif

}

uint16_t pwmData[(24*MAX_LED)+50];

void WS2812_Send (void)
{
	uint32_t indx=0;
	uint32_t color;


	for (int i= 0; i<MAX_LED; i++)
	{
#if USE_BRIGHTNESS
		color = ((LED_Mod[i][1]<<16) | (LED_Mod[i][2]<<8) | (LED_Mod[i][3]));
#else
		color = ((LED_Data[i][1]<<16) | (LED_Data[i][2]<<8) | (LED_Data[i][3]));
#endif

		for (int i=23; i>=0; i--)
		{
			if (color&(1<<i))
			{
				pwmData[indx] = 60;  // 2/3 of 90
			}

			else pwmData[indx] = 30;  // 1/3 of 90

			indx++;
		}

	}

	for (int i=0; i<50; i++)
	{
		pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_4, (uint32_t *)pwmData, indx);
	while (!datasentflag){};
	datasentflag = 0;
}

void Reset_LED (void)
{
	for (int i=0; i<MAX_LED; i++)
	{
		LED_Data[i][0] = i;
		LED_Data[i][1] = 0;
		LED_Data[i][2] = 0;
		LED_Data[i][3] = 0;
	}
}

uint16_t effStep = 0;

void setTailligght (int light, int R, int G, int B, int bright){
	  // Tail light
	  Set_LED(light, R, G, B);
	  Set_Brightness(bright);
	  WS2812_Send();
	  HAL_Delay(10);
	  // Tail light
}

// End LED

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if (GPIO_Pin==VL53L1X_INT_Pin)
//	{
//		IntCount++;
//	}
//}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	// VL53L1X variable references
	uint8_t buff[50];
	VL53L1_RangingMeasurementData_t RangingData;
	VL53L1_Dev_t  vl53l1_c; // center module
	VL53L1_DEV    Dev = &vl53l1_c;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FSMC_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // LCD Screen init
  LCD_INIT();

  // Init VL53L1X ToF
  Dev->I2cHandle = &hi2c2;
  Dev->I2cDevAddr = 0x52;

  HAL_GPIO_WritePin(XSHUT_GPIO_Port, XSHUT_Pin, GPIO_PIN_RESET);
  HAL_Delay(2); // 2ms reset time
  HAL_GPIO_WritePin(XSHUT_GPIO_Port, XSHUT_Pin, GPIO_PIN_SET);
  HAL_Delay(2);

  //  /*-[ I2C Bus Scanning ]-*/
      uint8_t i=0, ret;
      char text[100];
      for(i=1; i<128; i++)
      {
          ret = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(i<<1), 3, 5);
          if (ret != HAL_OK) /* No ACK Received At That Address */
          {
        	  LCD_DrawString(105, 125, "Loading........");
        	  Delay(10000);
        	  //println(&huart1,text);
          }
          else if(ret == HAL_OK)
          {
        	  sprintf(text, "Address: %x", i << 1);
        	  if (i == 0x40){
        		  LCD_DrawString(105, 125, "VL53L1X ToF port");
        	  }
        	  if (i == 103){
        		  LCD_DrawString(105, 125, "MPU6050     port");
        	  }
        	  LCD_DrawString(85, 125, text);
        	  Delay(10000000);
          }
      }

      Delay(10000);
//  /*** Initialize GPIO expanders ***/
////  // Unused GPIO should be configured as outputs to minimize the power consumption
//  buff[0] = 0x14; // GPDR (GPIO set direction register)
//  buff[1] = 0xFF; // GPIO_0 - GPIO_7
//  buff[2] = 0xFF; // GPIO_8 - GPIO_15
//  HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_1_ADDR, buff, 3, 0xFFFF );
//  HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_2_ADDR, buff, 3, 0xFFFF );
//
//  // clear XSHUT (disable center module) -> expander 1, GPIO_15
//  buff[0] = 0x13; // GPSR + 1 ( GPIO set pin state register)
//  HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_1_ADDR, buff, 1, 0xFFFF );
//  HAL_I2C_Master_Receive( &hi2c1, EXPANDER_1_ADDR, buff, 1, 0xFFFF );
//  buff[1] = buff[0] & ~( 1 << ( 15 - 8 ) ); // clear GPIO_15
//  buff[0] = 0x13; // GPSR + 1 ( GPIO set pin state register)
//  HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_1_ADDR, buff, 2, 0xFFFF );
//
//  HAL_Delay( 2 ); // 2ms reset time
//
//  // set XSHUT (enable center module) -> expander 1, GPIO_15
//  buff[0] = 0x13; // GPSR + 1 ( GPIO set pin state)
//  HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_1_ADDR, buff, 1, 0xFFFF );
//  HAL_I2C_Master_Receive( &hi2c1, EXPANDER_1_ADDR, buff, 1, 0xFFFF );
//  buff[1] = buff[0] | ( 1 << ( 15 - 8 ) ); // set GPIO_15
//  buff[0] = 0x13; // GPSR + 1 ( GPIO set pin state register)
//  HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_1_ADDR, buff, 2, 0xFFFF );

  HAL_Delay( 2 );
  VL53L1_WaitDeviceBooted( Dev );
  VL53L1_DataInit( Dev );
  VL53L1_StaticInit( Dev );
  VL53L1_SetDistanceMode( Dev, VL53L1_DISTANCEMODE_LONG );
  VL53L1_SetMeasurementTimingBudgetMicroSeconds( Dev, 50000 );
  VL53L1_SetInterMeasurementPeriodMilliSeconds( Dev, 500 );
  VL53L1_StartMeasurement( Dev );

  // End VL53L1X init

  // MPU6050 Init

  MPU6050_Initialize(&hi2c2);
  MPU6050_SetScaleAccelRange(&hi2c2, MPU6050_ACCEL_RANGE_8_G);
  MPU6050_SetScaleGyroRange(&hi2c2, MPU6050_GYRO_RANGE_2000_DEG);

  // MPU6050 Init end

  macXPT2046_CS_DISABLE();
//  __HAL_RCC_I2C2_CLK_DISABLE();
//  __HAL_RCC_FSMC_CLK_ENABLE();

//  LCD_INIT();

   Delay(100000);

  LCD_Clear (50, 80, 140, 70, RED);
  LCD_DrawString(65, 105, "  ISSAC  DEMO ");
  HAL_Delay(1000);

  //Touchscreen calibration

  while( ! XPT2046_Touch_Calibrate () );

  LCD_GramScan ( 1 );
  LCD_Clear ( 0, 0, 240, 320, WHITE );
  LCD_Clear ( 90,  282,  60, 60, BLUE	);
  LCD_Clear ( 0,  282,  95, 38,  YELLOW);
  LCD_Clear ( 150,  282,  95, 38, YELLOW);
 // LCD_DrawString(20, 280, "Turn Left");

  //Button UI
  LCD_DrawString_Color (35, 285, "<---", YELLOW, BLACK );
  LCD_DrawString_Color (170, 285, "--->", YELLOW, BLACK );

  LCD_DrawEllipse ( 120, 100, 40, 20, RED);
  char buf[9];
//  char text[10];

  // MPU variables
  float Ax, Ay, Az;
//  float Gx, Gy, Gz;

  strType_XPT2046_Coordinate touchCoordinate[4];
  int touchFlag = 0;
  int SafetyStatus = 0;	// 0: Save, 1: Caution, 2: Response needed, 3: Danger


  /*
   * Init of RPM Counter with timer and A3144 Hall effect sensor
   * */
  a3144_Init();
  float rotSpeed = 0;
  float rpm = 0;
  float tempSpeed = 0;
  int Rangecount = 0;

  // Reset Previus Tail lights
  Reset_LED();
  WS2812_Send();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  MPU6050_Read_Gyro(&hi2c2);
	  MPU6050_Read_Accel(&hi2c2);


	  Ax = MPU6050_Ax;
	  Ay = MPU6050_Ay;
	  Az = MPU6050_Az;

//	  Gx = MPU6050_Gx;
//	  Gy = MPU6050_Gy;
//	  Gz = MPU6050_Gz;

	  if (Ay < -0.4){
	 		  LCD_DrawString(80, 140, "Pitch Down");
	 	  } else if (Ay > 0.4){
	 		  LCD_DrawString(80, 140, " Pitch up ");
	 	  } else{
	 		  LCD_DrawString(80, 140, "No Pitch  ");
	 	  }

	 	  if (Ax < -0.5){
	 		  LCD_DrawString(90, 160, " Right ");
	 	  } else if (Ax > 0.5){
	 		  LCD_DrawString(90, 160, " Left ");
	 	  }	else{
	 		  LCD_DrawString(90, 160, " Still ");
	 	  }

	 	  if (Az < 0) {
	 		  LCD_DrawString_Color(90, 180, " Crash?? ", RED, BLACK);
	 		  for (int i =0; i < 14; i++){
	 			 setTailligght(i, 255, 0, 0, 10);
	 		  }
	 		  SafetyStatus = 2;
	 	  }


	 	  HAL_Delay(200);

	 	  if ( touchFlag == 0 )
	 	  {
	 		  if ( XPT2046_Get_TouchedPoint ( & touchCoordinate, & strXPT2046_TouchPara ) )
	 		  {
	 			  if ( ( touchCoordinate->y > 282 ) && ( touchCoordinate->y < 400) )
	 			  {
	 				  if ( ( touchCoordinate->x > 95 ) && ( touchCoordinate->x < 145 ) )
	 				  {
	 					  LCD_DrawString(71, 200, "     Reset    ");
	 					  SafetyStatus = 0;
	 					  LCD_Clear(90, 180, 90, 20, WHITE);
	 					  LCD_Clear(70, 200, 90, 20, WHITE);
	 					  Reset_LED();
	 					  WS2812_Send();
	 					  Rangecount = 0;
	 				  }

	 				  if (touchCoordinate->x < 90)
	 				  {
	 					  SafetyStatus = 1;
	 					  LCD_DrawString(71, 200, "Turning Left! ");
	 					  setTailligght(0, 255, 255, 0, 10);
	 					  setTailligght(1, 255, 255, 0, 10);
	 					  setTailligght(2, 255, 255, 0, 10);
	 				  }

	 				  if (touchCoordinate->x > 150)
	 				  {
	 					  SafetyStatus = 1;
	 					  LCD_DrawString(71, 200, "Turning Right!");
	 					  setTailligght(11, 255, 255, 0, 10);
	 					  setTailligght(12, 255, 255, 0, 10);
	 					  setTailligght(13, 255, 255, 0, 10);
	 				  }
	 			  }

	 		  }
	 		  touchFlag = 1;
	 	  }
	 	  else {
	 //		  LCD_DrawString(70, 200, "..............");
	 		  touchFlag = 0;
	 	  }
	 	  HAL_Delay(50);



		  rpm = GetRotationSpeed(); // RPM not speed
		  if (rpm < 1){
			  rpm = 0;
		  }
		  sprintf(buf, "%0.2f RPM", rpm);
		  LCD_DrawString(90, 90, buf);

		  rotSpeed = rpm * 0.737 * 3.14 / 60;
		  sprintf(buf, "%0.2f m/s", rotSpeed);
		  LCD_DrawString(90, 120, buf);

		  if (tempSpeed > rotSpeed * 0.6){
			  setTailligght(4, 255, 0, 0, 10);
			  setTailligght(5, 255, 0, 0, 10);
			  setTailligght(6, 255, 0, 0, 10);
			  setTailligght(7, 255, 0, 0, 10);
			  setTailligght(8, 255, 0, 0, 10);
			  setTailligght(9, 255, 0, 0, 10);
			  setTailligght(10, 255, 0, 0, 10);
		  }
		  else {
			  setTailligght(4, 0, 0, 0, 1);
			  setTailligght(5, 0, 0, 0, 1);
			  setTailligght(6, 0, 0, 0, 1);
			  setTailligght(7, 0, 0, 0, 1);
			  setTailligght(8, 0, 0, 0, 1);
			  setTailligght(9, 0, 0, 0, 1);
			  setTailligght(10, 0, 0, 0, 1);
		  }

		  tempSpeed = rotSpeed;

		  // Get Distance Sensor Data
		  VL53L1_WaitMeasurementDataReady( Dev );

		  VL53L1_GetRangingMeasurementData( Dev, &RangingData );


		  if(RangingData.RangeMilliMeter < 2000){
			  Rangecount++;
			  if(Rangecount == 3){
				  LCD_DrawString_Color (80, 200, "REAR ALERT!!", RED, BLACK );
				  Rangecount = 0;
			  }
		  }
		  else if(RangingData.RangeMilliMeter > 2000){
			  Rangecount = 0;
		  }


		  VL53L1_ClearInterruptAndStartMeasurement( Dev );
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 72-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 89;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, XPT2046_SPI_MOSI_Pin|LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LCD_BL_Pin|XPT2046_SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(XSHUT_GPIO_Port, XSHUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(XPT2046_SPI_CLK_GPIO_Port, XPT2046_SPI_CLK_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : XPT2046_SPI_MOSI_Pin XPT2046_SPI_CLK_Pin LCD_RST_Pin */
  GPIO_InitStruct.Pin = XPT2046_SPI_MOSI_Pin|XPT2046_SPI_CLK_Pin|LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : XPT2046_SPI_MISO_Pin */
  GPIO_InitStruct.Pin = XPT2046_SPI_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(XPT2046_SPI_MISO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : A3144_Pin */
  GPIO_InitStruct.Pin = A3144_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(A3144_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_BL_Pin XPT2046_SPI_CS_Pin */
  GPIO_InitStruct.Pin = LCD_BL_Pin|XPT2046_SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : XSHUT_Pin */
  GPIO_InitStruct.Pin = XSHUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(XSHUT_GPIO_Port, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};
  FSMC_NORSRAM_TimingTypeDef ExtTiming = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_ENABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 0;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 26;
  Timing.BusTurnAroundDuration = 0;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */
  ExtTiming.AddressSetupTime = 0;
  ExtTiming.AddressHoldTime = 15;
  ExtTiming.DataSetupTime = 1;
  ExtTiming.BusTurnAroundDuration = 0;
  ExtTiming.CLKDivision = 16;
  ExtTiming.DataLatency = 17;
  ExtTiming.AccessMode = FSMC_ACCESS_MODE_A;

  if (HAL_SRAM_Init(&hsram1, &Timing, &ExtTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Disconnect NADV
  */

  __HAL_AFIO_FSMCNADV_DISCONNECTED();

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
