################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Library_files/HAL_API_for_HTS221/HAL_I2C_anklabs.c 

OBJS += \
./Core/Library_files/HAL_API_for_HTS221/HAL_I2C_anklabs.o 

C_DEPS += \
./Core/Library_files/HAL_API_for_HTS221/HAL_I2C_anklabs.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Library_files/HAL_API_for_HTS221/%.o Core/Library_files/HAL_API_for_HTS221/%.su: ../Core/Library_files/HAL_API_for_HTS221/%.c Core/Library_files/HAL_API_for_HTS221/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WL55xx -c -I../Core/Inc -I"C:/Users/Hela/OneDrive/Bureau/stm32WL-bareMetal_workspace/ht221/CM4/Core/Library_files" -I"C:/Users/Hela/OneDrive/Bureau/stm32WL-bareMetal_workspace/ht221/CM4/Core/Library_files/HAL_API_for_HTS221" -I../../Drivers/STM32WLxx_HAL_Driver/Inc -I../../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Library_files-2f-HAL_API_for_HTS221

clean-Core-2f-Library_files-2f-HAL_API_for_HTS221:
	-$(RM) ./Core/Library_files/HAL_API_for_HTS221/HAL_I2C_anklabs.d ./Core/Library_files/HAL_API_for_HTS221/HAL_I2C_anklabs.o ./Core/Library_files/HAL_API_for_HTS221/HAL_I2C_anklabs.su

.PHONY: clean-Core-2f-Library_files-2f-HAL_API_for_HTS221

