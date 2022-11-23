/*
 MPU6050.h - Header file for the MPU6050_gy-521 is placed under the MIT license
 Copyright (c) 2021 M.Ali Aslan -> "mehmet.ali.aslan.abc@gmail.com"


 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
*/

#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

extern I2C_HandleTypeDef hi2c2;

#ifdef __cplusplus
extern "C" {
#endif

//Variables//
#define MPU6050_ADDR			0xD0
#define MPU6050_SMPLRT_DIV		0x19
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_PWR_MGMT_1		0x6B
#define MPU6050_WHO_AM_I		0x75

#define MPU6050_TEMP_OUT_H      0x41
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_ACCEL_XOUT_H    0x3B

 int16_t MPU6050_Accel_X_RAW;
 int16_t MPU6050_Accel_Y_RAW;
 int16_t MPU6050_Accel_Z_RAW;
 float MPU6050_Ax;
 float MPU6050_Ay;
 float MPU6050_Az;

 int16_t MPU6050_Gyro_X_RAW;
 int16_t MPU6050_Gyro_Y_RAW;
 int16_t MPU6050_Gyro_Z_RAW;
 float MPU6050_Gx;
 float MPU6050_Gy;
 float MPU6050_Gz;

 float MPU6050_Temperature;

 uint8_t MPU6050_AFS_SEL; //accel
 uint8_t MPU6050_FS_SEL; //gyro
 uint8_t MPU6050_Dev_ID; //Device ID
///************************//// Variables end


/////functions////
void MPU6050_Initialize(I2C_HandleTypeDef *handle_i2c); ///default initialize

//**MPU6050_SetScaleAccelRange**//
typedef enum {
	MPU6050_ACCEL_RANGE_2_G = 0x00,  ///< +/- 2g
	MPU6050_ACCEL_RANGE_4_G = 0x08,  ///< +/- 4g
	MPU6050_ACCEL_RANGE_8_G = 0x10,  ///< +/- 8g
	MPU6050_ACCEL_RANGE_16_G = 0x18, ///< +/- 16g
} mpu6050_accel_range_t;
//***//
void MPU6050_SetScaleAccelRange(I2C_HandleTypeDef *handle_i2c,mpu6050_accel_range_t MPU6050_ACCEL_RANGE_X_G);
//

//
//**MPU6050_SetScaleGyroRange**//
typedef enum {
	MPU6050_GYRO_RANGE_250_DEG = 0x00,  ///< +/- 250 deg/s (default value)
	MPU6050_GYRO_RANGE_500_DEG = 0x08,  ///< +/- 500 deg/s
	MPU6050_GYRO_RANGE_1000_DEG = 0x10, ///< +/- 1000 deg/s
	MPU6050_GYRO_RANGE_2000_DEG = 0x18, ///< +/- 2000 deg/s
} MPU6050_gyro_range_t;
//***//
void MPU6050_SetScaleGyroRange(I2C_HandleTypeDef *handle_i2c,MPU6050_gyro_range_t MPU6050_GYRO_RANGE_XXX_DEG);

void MPU6050_Read_DeviceID(I2C_HandleTypeDef *handle_i2c);

void MPU6050_Read_Accel(I2C_HandleTypeDef *handle_i2c);

void MPU6050_Read_Gyro(I2C_HandleTypeDef *handle_i2c);

void MPU6050_Read_Temp(I2C_HandleTypeDef *handle_i2c);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */

