################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../\ drivers/Src/stm32fxx_gpio.c \
../\ drivers/Src/stm32fxx_spi.c 

OBJS += \
./\ drivers/Src/stm32fxx_gpio.o \
./\ drivers/Src/stm32fxx_spi.o 

C_DEPS += \
./\ drivers/Src/stm32fxx_gpio.d \
./\ drivers/Src/stm32fxx_spi.d 


# Each subdirectory must supply rules for building sources it contributes
\ drivers/Src/stm32fxx_gpio.o: ../\ drivers/Src/stm32fxx_gpio.c \ drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/Users/ADMIN/Desktop/Learning/Embedded-C-master/Workspace/stm32fxxdriver/ drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF" drivers/Src/stm32fxx_gpio.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
\ drivers/Src/stm32fxx_spi.o: ../\ drivers/Src/stm32fxx_spi.c \ drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/Users/ADMIN/Desktop/Learning/Embedded-C-master/Workspace/stm32fxxdriver/ drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF" drivers/Src/stm32fxx_spi.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean--20-drivers-2f-Src

clean--20-drivers-2f-Src:
	-$(RM) ./\ drivers/Src/stm32fxx_gpio.d ./\ drivers/Src/stm32fxx_gpio.o ./\ drivers/Src/stm32fxx_gpio.su ./\ drivers/Src/stm32fxx_spi.d ./\ drivers/Src/stm32fxx_spi.o ./\ drivers/Src/stm32fxx_spi.su

.PHONY: clean--20-drivers-2f-Src

