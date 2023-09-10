################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../UserLib/Src/CRC16\ CCITT.c \
../UserLib/Src/Full\ Duplex\ USART\ r0.c \
../UserLib/Src/MAX31855\ r0.c 

OBJS += \
./UserLib/Src/CRC16\ CCITT.o \
./UserLib/Src/Full\ Duplex\ USART\ r0.o \
./UserLib/Src/MAX31855\ r0.o 

C_DEPS += \
./UserLib/Src/CRC16\ CCITT.d \
./UserLib/Src/Full\ Duplex\ USART\ r0.d \
./UserLib/Src/MAX31855\ r0.d 


# Each subdirectory must supply rules for building sources it contributes
UserLib/Src/CRC16\ CCITT.o: ../UserLib/Src/CRC16\ CCITT.c UserLib/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../UserLib/Inc -I"../UserLib/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fcommon -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"UserLib/Src/CRC16 CCITT.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
UserLib/Src/Full\ Duplex\ USART\ r0.o: ../UserLib/Src/Full\ Duplex\ USART\ r0.c UserLib/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../UserLib/Inc -I"../UserLib/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fcommon -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"UserLib/Src/Full Duplex USART r0.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
UserLib/Src/MAX31855\ r0.o: ../UserLib/Src/MAX31855\ r0.c UserLib/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../UserLib/Inc -I"../UserLib/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fcommon -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"UserLib/Src/MAX31855 r0.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-UserLib-2f-Src

clean-UserLib-2f-Src:
	-$(RM) ./UserLib/Src/CRC16\ CCITT.cyclo ./UserLib/Src/CRC16\ CCITT.d ./UserLib/Src/CRC16\ CCITT.o ./UserLib/Src/CRC16\ CCITT.su ./UserLib/Src/Full\ Duplex\ USART\ r0.cyclo ./UserLib/Src/Full\ Duplex\ USART\ r0.d ./UserLib/Src/Full\ Duplex\ USART\ r0.o ./UserLib/Src/Full\ Duplex\ USART\ r0.su ./UserLib/Src/MAX31855\ r0.cyclo ./UserLib/Src/MAX31855\ r0.d ./UserLib/Src/MAX31855\ r0.o ./UserLib/Src/MAX31855\ r0.su

.PHONY: clean-UserLib-2f-Src

