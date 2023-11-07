################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Leo/STM32CubeIDE/new_workspace/GPIO_InfiniteLedToggling/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_gpio.c \
C:/Users/Leo/STM32CubeIDE/new_workspace/GPIO_InfiniteLedToggling/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_utils.c 

OBJS += \
./Drivers/STM32H7xx_LL_Driver/stm32h7xx_ll_gpio.o \
./Drivers/STM32H7xx_LL_Driver/stm32h7xx_ll_utils.o 

C_DEPS += \
./Drivers/STM32H7xx_LL_Driver/stm32h7xx_ll_gpio.d \
./Drivers/STM32H7xx_LL_Driver/stm32h7xx_ll_utils.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32H7xx_LL_Driver/stm32h7xx_ll_gpio.o: C:/Users/Leo/STM32CubeIDE/new_workspace/GPIO_InfiniteLedToggling/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_gpio.c Drivers/STM32H7xx_LL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32H7A3xxQ -DUSE_FULL_LL_DRIVER -DDEBUG -c -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32H7xx_LL_Driver/stm32h7xx_ll_utils.o: C:/Users/Leo/STM32CubeIDE/new_workspace/GPIO_InfiniteLedToggling/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_utils.c Drivers/STM32H7xx_LL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32H7A3xxQ -DUSE_FULL_LL_DRIVER -DDEBUG -c -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-STM32H7xx_LL_Driver

clean-Drivers-2f-STM32H7xx_LL_Driver:
	-$(RM) ./Drivers/STM32H7xx_LL_Driver/stm32h7xx_ll_gpio.cyclo ./Drivers/STM32H7xx_LL_Driver/stm32h7xx_ll_gpio.d ./Drivers/STM32H7xx_LL_Driver/stm32h7xx_ll_gpio.o ./Drivers/STM32H7xx_LL_Driver/stm32h7xx_ll_gpio.su ./Drivers/STM32H7xx_LL_Driver/stm32h7xx_ll_utils.cyclo ./Drivers/STM32H7xx_LL_Driver/stm32h7xx_ll_utils.d ./Drivers/STM32H7xx_LL_Driver/stm32h7xx_ll_utils.o ./Drivers/STM32H7xx_LL_Driver/stm32h7xx_ll_utils.su

.PHONY: clean-Drivers-2f-STM32H7xx_LL_Driver

