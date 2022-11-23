################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Library_Code/MPU6050.c 

OBJS += \
./Library_Code/MPU6050.o 

C_DEPS += \
./Library_Code/MPU6050.d 


# Each subdirectory must supply rules for building sources it contributes
Library_Code/%.o Library_Code/%.su: ../Library_Code/%.c Library_Code/subdir.mk
	arm-none-eabi-gcc -fcommon "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Library_Code -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"/Users/kennethlok/STM32CubeIDE/workspace_1.9.0/Intelligent-Safety-System-Assisting-Cyclist/Library_Code" -include../Library_Code/MPU6050.h -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Library_Code

clean-Library_Code:
	-$(RM) ./Library_Code/MPU6050.d ./Library_Code/MPU6050.o ./Library_Code/MPU6050.su

.PHONY: clean-Library_Code

