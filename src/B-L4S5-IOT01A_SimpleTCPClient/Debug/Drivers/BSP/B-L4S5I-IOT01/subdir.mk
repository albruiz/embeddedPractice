################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.c \
../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.c \
../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.c 

OBJS += \
./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.o \
./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.o \
./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.o 

C_DEPS += \
./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.d \
./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.d \
./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/B-L4S5I-IOT01/%.o Drivers/BSP/B-L4S5I-IOT01/%.su Drivers/BSP/B-L4S5I-IOT01/%.cyclo: ../Drivers/BSP/B-L4S5I-IOT01/%.c Drivers/BSP/B-L4S5I-IOT01/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/ST/workspace/embeddedPractice/src/B-L4S5-IOT01A_SimpleTCPClient/Drivers/BSP/B-L4S5I-IOT01" -I"C:/ST/workspace/embeddedPractice/src/B-L4S5-IOT01A_SimpleTCPClient/Drivers/BSP/Components/Common" -I"C:/ST/workspace/embeddedPractice/src/B-L4S5-IOT01A_SimpleTCPClient/Drivers/BSP/Components/es_wifi" -I"C:/ST/workspace/embeddedPractice/src/B-L4S5-IOT01A_SimpleTCPClient/Drivers/BSP/Components/hts221" -I"C:/ST/workspace/embeddedPractice/src/B-L4S5-IOT01A_SimpleTCPClient/Drivers/BSP/Components/st25dv" -I"C:/ST/workspace/embeddedPractice/src/B-L4S5-IOT01A_SimpleTCPClient/Middlewares/Third_Party/Wifi" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-B-2d-L4S5I-2d-IOT01

clean-Drivers-2f-BSP-2f-B-2d-L4S5I-2d-IOT01:
	-$(RM) ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.cyclo ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.d ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.o ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.su ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.cyclo ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.d ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.o ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.su ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.cyclo ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.d ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.o ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.su

.PHONY: clean-Drivers-2f-BSP-2f-B-2d-L4S5I-2d-IOT01

