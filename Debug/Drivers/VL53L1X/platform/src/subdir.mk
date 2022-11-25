################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/VL53L1X/platform/src/vl53l1_platform.c \
../Drivers/VL53L1X/platform/src/vl53l1_platform_log.c 

OBJS += \
./Drivers/VL53L1X/platform/src/vl53l1_platform.o \
./Drivers/VL53L1X/platform/src/vl53l1_platform_log.o 

C_DEPS += \
./Drivers/VL53L1X/platform/src/vl53l1_platform.d \
./Drivers/VL53L1X/platform/src/vl53l1_platform_log.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/VL53L1X/platform/src/%.o Drivers/VL53L1X/platform/src/%.su: ../Drivers/VL53L1X/platform/src/%.c Drivers/VL53L1X/platform/src/subdir.mk
	arm-none-eabi-gcc -fcommon "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Library_Code -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"/Users/kennethlok/STM32CubeIDE/workspace_1.9.0/Intelligent-Safety-System-Assisting-Cyclist/Library_Code" -I"/Users/kennethlok/STM32CubeIDE/workspace_1.9.0/Intelligent-Safety-System-Assisting-Cyclist/Drivers/VL53L1X/core/inc" -I"/Users/kennethlok/STM32CubeIDE/workspace_1.9.0/Intelligent-Safety-System-Assisting-Cyclist/Drivers/VL53L1X/platform/inc" -include../Library_Code/MPU6050.h -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-VL53L1X-2f-platform-2f-src

clean-Drivers-2f-VL53L1X-2f-platform-2f-src:
	-$(RM) ./Drivers/VL53L1X/platform/src/vl53l1_platform.d ./Drivers/VL53L1X/platform/src/vl53l1_platform.o ./Drivers/VL53L1X/platform/src/vl53l1_platform.su ./Drivers/VL53L1X/platform/src/vl53l1_platform_log.d ./Drivers/VL53L1X/platform/src/vl53l1_platform_log.o ./Drivers/VL53L1X/platform/src/vl53l1_platform_log.su

.PHONY: clean-Drivers-2f-VL53L1X-2f-platform-2f-src

