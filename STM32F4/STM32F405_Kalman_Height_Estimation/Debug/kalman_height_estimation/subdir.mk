################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Data/Firmware/Demo_STM32/Components/kalman_height_estimation/kalman_height_estimation.c 

OBJS += \
./kalman_height_estimation/kalman_height_estimation.o 

C_DEPS += \
./kalman_height_estimation/kalman_height_estimation.d 


# Each subdirectory must supply rules for building sources it contributes
kalman_height_estimation/kalman_height_estimation.o: C:/Data/Firmware/Demo_STM32/Components/kalman_height_estimation/kalman_height_estimation.c kalman_height_estimation/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Data/Firmware/Demo_STM32/STM32F4/STM32F405_Kalman_Height_Estimation/../../Components/mcu_port" -I"C:/Data/Firmware/Demo_STM32/STM32F4/STM32F405_Kalman_Height_Estimation/../../Components/bmp280" -I"C:/Data/Firmware/Demo_STM32/STM32F4/STM32F405_Kalman_Height_Estimation/../../Components/kalman_height_estimation" -I"C:/Data/Firmware/Demo_STM32/STM32F4/STM32F405_Kalman_Height_Estimation/../../Components/CLinearAlgebra" -I"C:/Data/Firmware/Demo_STM32/STM32F4/STM32F405_Kalman_Height_Estimation/../../Components/icm42688" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-kalman_height_estimation

clean-kalman_height_estimation:
	-$(RM) ./kalman_height_estimation/kalman_height_estimation.cyclo ./kalman_height_estimation/kalman_height_estimation.d ./kalman_height_estimation/kalman_height_estimation.o ./kalman_height_estimation/kalman_height_estimation.su

.PHONY: clean-kalman_height_estimation

