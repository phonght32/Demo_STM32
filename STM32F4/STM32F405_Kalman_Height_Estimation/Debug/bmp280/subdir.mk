################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Data/Firmware/Demo_STM32/Components/bmp280/bmp280.c 

OBJS += \
./bmp280/bmp280.o 

C_DEPS += \
./bmp280/bmp280.d 


# Each subdirectory must supply rules for building sources it contributes
bmp280/bmp280.o: C:/Data/Firmware/Demo_STM32/Components/bmp280/bmp280.c bmp280/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Data/Firmware/Demo_STM32/STM32F4/STM32F405_Kalman_Height_Estimation/../../Components/mcu_port" -I"C:/Data/Firmware/Demo_STM32/STM32F4/STM32F405_Kalman_Height_Estimation/../../Components/bmp280" -I"C:/Data/Firmware/Demo_STM32/STM32F4/STM32F405_Kalman_Height_Estimation/../../Components/kalman_height_estimation" -I"C:/Data/Firmware/Demo_STM32/STM32F4/STM32F405_Kalman_Height_Estimation/../../Components/CLinearAlgebra" -I"C:/Data/Firmware/Demo_STM32/STM32F4/STM32F405_Kalman_Height_Estimation/../../Components/icm42688" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-bmp280

clean-bmp280:
	-$(RM) ./bmp280/bmp280.cyclo ./bmp280/bmp280.d ./bmp280/bmp280.o ./bmp280/bmp280.su

.PHONY: clean-bmp280

