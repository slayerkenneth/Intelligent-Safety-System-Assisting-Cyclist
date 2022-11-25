################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/VL53L1X/core/src/vl53l1_api.c \
../Drivers/VL53L1X/core/src/vl53l1_api_calibration.c \
../Drivers/VL53L1X/core/src/vl53l1_api_core.c \
../Drivers/VL53L1X/core/src/vl53l1_api_debug.c \
../Drivers/VL53L1X/core/src/vl53l1_api_preset_modes.c \
../Drivers/VL53L1X/core/src/vl53l1_api_strings.c \
../Drivers/VL53L1X/core/src/vl53l1_core.c \
../Drivers/VL53L1X/core/src/vl53l1_core_support.c \
../Drivers/VL53L1X/core/src/vl53l1_error_strings.c \
../Drivers/VL53L1X/core/src/vl53l1_register_funcs.c \
../Drivers/VL53L1X/core/src/vl53l1_silicon_core.c \
../Drivers/VL53L1X/core/src/vl53l1_wait.c 

OBJS += \
./Drivers/VL53L1X/core/src/vl53l1_api.o \
./Drivers/VL53L1X/core/src/vl53l1_api_calibration.o \
./Drivers/VL53L1X/core/src/vl53l1_api_core.o \
./Drivers/VL53L1X/core/src/vl53l1_api_debug.o \
./Drivers/VL53L1X/core/src/vl53l1_api_preset_modes.o \
./Drivers/VL53L1X/core/src/vl53l1_api_strings.o \
./Drivers/VL53L1X/core/src/vl53l1_core.o \
./Drivers/VL53L1X/core/src/vl53l1_core_support.o \
./Drivers/VL53L1X/core/src/vl53l1_error_strings.o \
./Drivers/VL53L1X/core/src/vl53l1_register_funcs.o \
./Drivers/VL53L1X/core/src/vl53l1_silicon_core.o \
./Drivers/VL53L1X/core/src/vl53l1_wait.o 

C_DEPS += \
./Drivers/VL53L1X/core/src/vl53l1_api.d \
./Drivers/VL53L1X/core/src/vl53l1_api_calibration.d \
./Drivers/VL53L1X/core/src/vl53l1_api_core.d \
./Drivers/VL53L1X/core/src/vl53l1_api_debug.d \
./Drivers/VL53L1X/core/src/vl53l1_api_preset_modes.d \
./Drivers/VL53L1X/core/src/vl53l1_api_strings.d \
./Drivers/VL53L1X/core/src/vl53l1_core.d \
./Drivers/VL53L1X/core/src/vl53l1_core_support.d \
./Drivers/VL53L1X/core/src/vl53l1_error_strings.d \
./Drivers/VL53L1X/core/src/vl53l1_register_funcs.d \
./Drivers/VL53L1X/core/src/vl53l1_silicon_core.d \
./Drivers/VL53L1X/core/src/vl53l1_wait.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/VL53L1X/core/src/%.o Drivers/VL53L1X/core/src/%.su: ../Drivers/VL53L1X/core/src/%.c Drivers/VL53L1X/core/src/subdir.mk
	arm-none-eabi-gcc -fcommon "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Library_Code -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"/Users/kennethlok/STM32CubeIDE/workspace_1.9.0/Intelligent-Safety-System-Assisting-Cyclist/Library_Code" -I"/Users/kennethlok/STM32CubeIDE/workspace_1.9.0/Intelligent-Safety-System-Assisting-Cyclist/Drivers/VL53L1X/core/inc" -I"/Users/kennethlok/STM32CubeIDE/workspace_1.9.0/Intelligent-Safety-System-Assisting-Cyclist/Drivers/VL53L1X/platform/inc" -include../Library_Code/MPU6050.h -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-VL53L1X-2f-core-2f-src

clean-Drivers-2f-VL53L1X-2f-core-2f-src:
	-$(RM) ./Drivers/VL53L1X/core/src/vl53l1_api.d ./Drivers/VL53L1X/core/src/vl53l1_api.o ./Drivers/VL53L1X/core/src/vl53l1_api.su ./Drivers/VL53L1X/core/src/vl53l1_api_calibration.d ./Drivers/VL53L1X/core/src/vl53l1_api_calibration.o ./Drivers/VL53L1X/core/src/vl53l1_api_calibration.su ./Drivers/VL53L1X/core/src/vl53l1_api_core.d ./Drivers/VL53L1X/core/src/vl53l1_api_core.o ./Drivers/VL53L1X/core/src/vl53l1_api_core.su ./Drivers/VL53L1X/core/src/vl53l1_api_debug.d ./Drivers/VL53L1X/core/src/vl53l1_api_debug.o ./Drivers/VL53L1X/core/src/vl53l1_api_debug.su ./Drivers/VL53L1X/core/src/vl53l1_api_preset_modes.d ./Drivers/VL53L1X/core/src/vl53l1_api_preset_modes.o ./Drivers/VL53L1X/core/src/vl53l1_api_preset_modes.su ./Drivers/VL53L1X/core/src/vl53l1_api_strings.d ./Drivers/VL53L1X/core/src/vl53l1_api_strings.o ./Drivers/VL53L1X/core/src/vl53l1_api_strings.su ./Drivers/VL53L1X/core/src/vl53l1_core.d ./Drivers/VL53L1X/core/src/vl53l1_core.o ./Drivers/VL53L1X/core/src/vl53l1_core.su ./Drivers/VL53L1X/core/src/vl53l1_core_support.d ./Drivers/VL53L1X/core/src/vl53l1_core_support.o ./Drivers/VL53L1X/core/src/vl53l1_core_support.su ./Drivers/VL53L1X/core/src/vl53l1_error_strings.d ./Drivers/VL53L1X/core/src/vl53l1_error_strings.o ./Drivers/VL53L1X/core/src/vl53l1_error_strings.su ./Drivers/VL53L1X/core/src/vl53l1_register_funcs.d ./Drivers/VL53L1X/core/src/vl53l1_register_funcs.o ./Drivers/VL53L1X/core/src/vl53l1_register_funcs.su ./Drivers/VL53L1X/core/src/vl53l1_silicon_core.d ./Drivers/VL53L1X/core/src/vl53l1_silicon_core.o ./Drivers/VL53L1X/core/src/vl53l1_silicon_core.su ./Drivers/VL53L1X/core/src/vl53l1_wait.d ./Drivers/VL53L1X/core/src/vl53l1_wait.o ./Drivers/VL53L1X/core/src/vl53l1_wait.su

.PHONY: clean-Drivers-2f-VL53L1X-2f-core-2f-src

