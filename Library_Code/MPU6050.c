/*
 MPU6050.cpp - Class file for the MPU6050_gy-521 is placed under the MIT license
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

/* Includes */
#include "MPU6050.h"
#include "stm32f1xx.h"

/////functions////
void MPU6050_Initialize(I2C_HandleTypeDef *handle_i2c) {
	uint8_t check;
	uint8_t Data;

	HAL_I2C_Mem_Read(handle_i2c, MPU6050_ADDR, MPU6050_WHO_AM_I, 1, &check, 1,10000);

	if (check == 0x68) {

		Data = 0;
		HAL_I2C_Mem_Write(handle_i2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 1, &Data, 1,10000);

		Data = 0x07;
		HAL_I2C_Mem_Write(handle_i2c, MPU6050_ADDR, MPU6050_SMPLRT_DIV, 1, &Data, 1,10000);

		HAL_I2C_Mem_Write(handle_i2c, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 1, MPU6050_ACCEL_RANGE_2_G, 1,10000);
		MPU6050_AFS_SEL=MPU6050_ACCEL_RANGE_2_G;

		HAL_I2C_Mem_Write(handle_i2c, MPU6050_ADDR, MPU6050_GYRO_CONFIG, 1, MPU6050_GYRO_RANGE_250_DEG, 1, 10000);
		MPU6050_FS_SEL=MPU6050_GYRO_RANGE_250_DEG;
	}

}

void MPU6050_Read_DeviceID(I2C_HandleTypeDef *handle_i2c){
    HAL_I2C_Mem_Read(handle_i2c, MPU6050_ADDR, MPU6050_WHO_AM_I, 1, &MPU6050_Dev_ID, 1, 10000);
}

void MPU6050_SetScaleAccelRange(I2C_HandleTypeDef *handle_i2c,mpu6050_accel_range_t MPU6050_ACCEL_RANGE_X_G)
{
	HAL_I2C_Mem_Write(handle_i2c, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 1,&MPU6050_ACCEL_RANGE_X_G, 1,10000);
	MPU6050_AFS_SEL=MPU6050_ACCEL_RANGE_X_G;
}


void MPU6050_SetScaleGyroRange(I2C_HandleTypeDef *handle_i2c,MPU6050_gyro_range_t MPU6050_GYRO_RANGE_XXX_DEG)
{
	HAL_I2C_Mem_Write(handle_i2c, MPU6050_ADDR, MPU6050_GYRO_CONFIG, 1,&MPU6050_GYRO_RANGE_XXX_DEG, 1,10000);
	MPU6050_FS_SEL=MPU6050_GYRO_RANGE_XXX_DEG;
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *handle_i2c) {
	uint8_t Rec_Data[6];

	HAL_I2C_Mem_Read(handle_i2c, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 1, Rec_Data, 6,10000);

	MPU6050_Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	MPU6050_Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	MPU6050_Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

	/*
	AFS_SEL Full Scale Range LSB Sensitivity
	0 ±2g 16384 LSB/g
	1 ±4g 8192 LSB/g
	2 ±8g 4096 LSB/g
	3 ±16g 2048 LSB/g
	 */
	switch (MPU6050_AFS_SEL) {
	case 0x00://0
		MPU6050_Ax = MPU6050_Accel_X_RAW / 16384.0;
		MPU6050_Ay = MPU6050_Accel_Y_RAW / 16384.0;
		MPU6050_Az = MPU6050_Accel_Z_RAW / 16384.0;

		break;
	case 0x08://1
		MPU6050_Ax = MPU6050_Accel_X_RAW / 8192.0;
		MPU6050_Ay = MPU6050_Accel_Y_RAW / 8192.0;
		MPU6050_Az = MPU6050_Accel_Z_RAW / 8192.0;

		break;
	case 0x10://2
		MPU6050_Ax = MPU6050_Accel_X_RAW / 4096.0;
		MPU6050_Ay = MPU6050_Accel_Y_RAW / 4096.0;
		MPU6050_Az = MPU6050_Accel_Z_RAW / 4096.0;

		break;
	case 0x18://3
		MPU6050_Ax = MPU6050_Accel_X_RAW / 2048.0;
		MPU6050_Ay = MPU6050_Accel_Y_RAW / 2048.0;
		MPU6050_Az = MPU6050_Accel_Z_RAW / 2048.0;

		break;
	}
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *handle_i2c) {
	uint8_t Rec_Data[6];

	HAL_I2C_Mem_Read(handle_i2c, MPU6050_ADDR, MPU6050_GYRO_XOUT_H, 1, Rec_Data, 6,10000);

	MPU6050_Gyro_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	MPU6050_Gyro_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	MPU6050_Gyro_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

	/*
	FS_SEL Full Scale Range LSB Sensitivity
	0 ± 250 °/s 131 LSB/°/s
	1 ± 500 °/s 65.5 LSB/°/s
	2 ± 1000 °/s 32.8 LSB/°/s
	3 ± 2000 °/s 16.4 LSB/°/s
	 */

	switch (MPU6050_FS_SEL) {
	case 0x00://0
		MPU6050_Gx = MPU6050_Gyro_X_RAW / 131.0;
		MPU6050_Gy = MPU6050_Gyro_Y_RAW / 131.0;
		MPU6050_Gz = MPU6050_Gyro_Z_RAW / 131.0;
		break;
	case 0x08://1
		MPU6050_Gx = MPU6050_Gyro_X_RAW / 65.5;
		MPU6050_Gy = MPU6050_Gyro_Y_RAW / 65.5;
		MPU6050_Gz = MPU6050_Gyro_Z_RAW / 65.5;
		break;
	case 0x10://2
		MPU6050_Gx = MPU6050_Gyro_X_RAW / 32.8;
		MPU6050_Gy = MPU6050_Gyro_Y_RAW / 32.8;
		MPU6050_Gz = MPU6050_Gyro_Z_RAW / 32.8;
		break;
	case 0x18://3
		MPU6050_Gx = MPU6050_Gyro_X_RAW / 16.4;
		MPU6050_Gy = MPU6050_Gyro_Y_RAW / 16.4;
		MPU6050_Gz = MPU6050_Gyro_Z_RAW / 16.4;
		break;
	}
}
void MPU6050_Read_Temp(I2C_HandleTypeDef *handle_i2c) {
    uint8_t Rec_Data[2];
    int16_t temp;

    HAL_I2C_Mem_Read(handle_i2c, MPU6050_ADDR, MPU6050_TEMP_OUT_H, 1, Rec_Data, 2, 10000);

    temp = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    MPU6050_Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
}
