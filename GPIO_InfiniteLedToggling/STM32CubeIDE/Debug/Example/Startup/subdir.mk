################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Example/Startup/startup_stm32h7a3zitxq.s 

OBJS += \
./Example/Startup/startup_stm32h7a3zitxq.o 

S_DEPS += \
./Example/Startup/startup_stm32h7a3zitxq.d 


# Each subdirectory must supply rules for building sources it contributes
Example/Startup/%.o: ../Example/Startup/%.s Example/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -g3 -c -I../ -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Example-2f-Startup

clean-Example-2f-Startup:
	-$(RM) ./Example/Startup/startup_stm32h7a3zitxq.d ./Example/Startup/startup_stm32h7a3zitxq.o

.PHONY: clean-Example-2f-Startup

