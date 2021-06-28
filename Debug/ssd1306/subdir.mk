################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ssd1306/FIRFilter.c \
../ssd1306/angles.c \
../ssd1306/mpu6050.c \
../ssd1306/ssd1306.c \
../ssd1306/ssd1306_fonts.c 

OBJS += \
./ssd1306/FIRFilter.o \
./ssd1306/angles.o \
./ssd1306/mpu6050.o \
./ssd1306/ssd1306.o \
./ssd1306/ssd1306_fonts.o 

C_DEPS += \
./ssd1306/FIRFilter.d \
./ssd1306/angles.d \
./ssd1306/mpu6050.d \
./ssd1306/ssd1306.d \
./ssd1306/ssd1306_fonts.d 


# Each subdirectory must supply rules for building sources it contributes
ssd1306/FIRFilter.o: ../ssd1306/FIRFilter.c ssd1306/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"/home/tomas/Dropbox/STM32CubeIDE/workspace_1.6.1/oled_experimenting/ssd1306" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ssd1306/FIRFilter.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
ssd1306/angles.o: ../ssd1306/angles.c ssd1306/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"/home/tomas/Dropbox/STM32CubeIDE/workspace_1.6.1/oled_experimenting/ssd1306" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ssd1306/angles.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
ssd1306/mpu6050.o: ../ssd1306/mpu6050.c ssd1306/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"/home/tomas/Dropbox/STM32CubeIDE/workspace_1.6.1/oled_experimenting/ssd1306" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ssd1306/mpu6050.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
ssd1306/ssd1306.o: ../ssd1306/ssd1306.c ssd1306/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"/home/tomas/Dropbox/STM32CubeIDE/workspace_1.6.1/oled_experimenting/ssd1306" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ssd1306/ssd1306.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
ssd1306/ssd1306_fonts.o: ../ssd1306/ssd1306_fonts.c ssd1306/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"/home/tomas/Dropbox/STM32CubeIDE/workspace_1.6.1/oled_experimenting/ssd1306" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ssd1306/ssd1306_fonts.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

