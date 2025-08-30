################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/App/Src/app_crc16.c \
../Core/App/Src/app_meta.c \
../Core/App/Src/app_proto.c \
../Core/App/Src/app_system.c \
../Core/App/Src/app_tasks.c \
../Core/App/Src/app_uart.c 

OBJS += \
./Core/App/Src/app_crc16.o \
./Core/App/Src/app_meta.o \
./Core/App/Src/app_proto.o \
./Core/App/Src/app_system.o \
./Core/App/Src/app_tasks.o \
./Core/App/Src/app_uart.o 

C_DEPS += \
./Core/App/Src/app_crc16.d \
./Core/App/Src/app_meta.d \
./Core/App/Src/app_proto.d \
./Core/App/Src/app_system.d \
./Core/App/Src/app_tasks.d \
./Core/App/Src/app_uart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/App/Src/%.o Core/App/Src/%.su Core/App/Src/%.cyclo: ../Core/App/Src/%.c Core/App/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I"D:/bootloaderF1_OS/stm32_app/Core/App/Inc" -I"D:/bootloaderF1_OS/stm32_app/Core/App/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-App-2f-Src

clean-Core-2f-App-2f-Src:
	-$(RM) ./Core/App/Src/app_crc16.cyclo ./Core/App/Src/app_crc16.d ./Core/App/Src/app_crc16.o ./Core/App/Src/app_crc16.su ./Core/App/Src/app_meta.cyclo ./Core/App/Src/app_meta.d ./Core/App/Src/app_meta.o ./Core/App/Src/app_meta.su ./Core/App/Src/app_proto.cyclo ./Core/App/Src/app_proto.d ./Core/App/Src/app_proto.o ./Core/App/Src/app_proto.su ./Core/App/Src/app_system.cyclo ./Core/App/Src/app_system.d ./Core/App/Src/app_system.o ./Core/App/Src/app_system.su ./Core/App/Src/app_tasks.cyclo ./Core/App/Src/app_tasks.d ./Core/App/Src/app_tasks.o ./Core/App/Src/app_tasks.su ./Core/App/Src/app_uart.cyclo ./Core/App/Src/app_uart.d ./Core/App/Src/app_uart.o ./Core/App/Src/app_uart.su

.PHONY: clean-Core-2f-App-2f-Src

