################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/lsm303c/lsm303c.c 

OBJS += \
./Drivers/BSP/Components/lsm303c/lsm303c.o 

C_DEPS += \
./Drivers/BSP/Components/lsm303c/lsm303c.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/lsm303c/%.o Drivers/BSP/Components/lsm303c/%.su Drivers/BSP/Components/lsm303c/%.cyclo: ../Drivers/BSP/Components/lsm303c/%.c Drivers/BSP/Components/lsm303c/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/Components -I../Drivers/BSP/B-L4S5I-IOT01 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-lsm303c

clean-Drivers-2f-BSP-2f-Components-2f-lsm303c:
	-$(RM) ./Drivers/BSP/Components/lsm303c/lsm303c.cyclo ./Drivers/BSP/Components/lsm303c/lsm303c.d ./Drivers/BSP/Components/lsm303c/lsm303c.o ./Drivers/BSP/Components/lsm303c/lsm303c.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-lsm303c

