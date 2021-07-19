################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/FIRFilter.c \
../Libraries/angles.c \
../Libraries/gui_multitool.c \
../Libraries/hc-sr04.c \
../Libraries/mpu6050.c \
../Libraries/ssd1306.c \
../Libraries/ssd1306_fonts.c 

OBJS += \
./Libraries/FIRFilter.o \
./Libraries/angles.o \
./Libraries/gui_multitool.o \
./Libraries/hc-sr04.o \
./Libraries/mpu6050.o \
./Libraries/ssd1306.o \
./Libraries/ssd1306_fonts.o 

C_DEPS += \
./Libraries/FIRFilter.d \
./Libraries/angles.d \
./Libraries/gui_multitool.d \
./Libraries/hc-sr04.d \
./Libraries/mpu6050.d \
./Libraries/ssd1306.d \
./Libraries/ssd1306_fonts.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/FIRFilter.o: ../Libraries/FIRFilter.c Libraries/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"/home/tomas/Dropbox/STM32CubeIDE/workspace_1.6.1/oled_accel/Libraries" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Libraries/FIRFilter.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Libraries/angles.o: ../Libraries/angles.c Libraries/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"/home/tomas/Dropbox/STM32CubeIDE/workspace_1.6.1/oled_accel/Libraries" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Libraries/angles.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Libraries/gui_multitool.o: ../Libraries/gui_multitool.c Libraries/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"/home/tomas/Dropbox/STM32CubeIDE/workspace_1.6.1/oled_accel/Libraries" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Libraries/gui_multitool.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Libraries/hc-sr04.o: ../Libraries/hc-sr04.c Libraries/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"/home/tomas/Dropbox/STM32CubeIDE/workspace_1.6.1/oled_accel/Libraries" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Libraries/hc-sr04.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Libraries/mpu6050.o: ../Libraries/mpu6050.c Libraries/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"/home/tomas/Dropbox/STM32CubeIDE/workspace_1.6.1/oled_accel/Libraries" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Libraries/mpu6050.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Libraries/ssd1306.o: ../Libraries/ssd1306.c Libraries/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"/home/tomas/Dropbox/STM32CubeIDE/workspace_1.6.1/oled_accel/Libraries" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Libraries/ssd1306.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Libraries/ssd1306_fonts.o: ../Libraries/ssd1306_fonts.c Libraries/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"/home/tomas/Dropbox/STM32CubeIDE/workspace_1.6.1/oled_accel/Libraries" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Libraries/ssd1306_fonts.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

