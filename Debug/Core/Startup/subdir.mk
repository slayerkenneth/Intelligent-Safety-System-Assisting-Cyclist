################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f103vetx.s 

OBJS += \
./Core/Startup/startup_stm32f103vetx.o 

S_DEPS += \
./Core/Startup/startup_stm32f103vetx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m3 -g3 -DDEBUG -c -I"/Users/kennethlok/STM32CubeIDE/workspace_1.9.0/Intelligent-Safety-System-Assisting-Cyclist/Drivers/VL53L1X/core/inc" -I"/Users/kennethlok/STM32CubeIDE/workspace_1.9.0/Intelligent-Safety-System-Assisting-Cyclist/Drivers/VL53L1X/platform/inc" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32f103vetx.d ./Core/Startup/startup_stm32f103vetx.o

.PHONY: clean-Core-2f-Startup

