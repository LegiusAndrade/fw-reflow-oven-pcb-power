################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../UserLib/Src/MAX31855.c 

OBJS += \
./UserLib/Src/MAX31855.o 

C_DEPS += \
./UserLib/Src/MAX31855.d 


# Each subdirectory must supply rules for building sources it contributes
UserLib/Src/%.o UserLib/Src/%.su UserLib/Src/%.cyclo: ../UserLib/Src/%.c UserLib/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../UserLib/Inc -I"../UserLib/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-UserLib-2f-Src

clean-UserLib-2f-Src:
	-$(RM) ./UserLib/Src/MAX31855.cyclo ./UserLib/Src/MAX31855.d ./UserLib/Src/MAX31855.o ./UserLib/Src/MAX31855.su

.PHONY: clean-UserLib-2f-Src

