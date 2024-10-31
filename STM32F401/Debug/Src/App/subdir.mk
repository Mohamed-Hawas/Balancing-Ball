################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/App/Balancing_Ball.c 

OBJS += \
./Src/App/Balancing_Ball.o 

C_DEPS += \
./Src/App/Balancing_Ball.d 


# Each subdirectory must supply rules for building sources it contributes
Src/App/Balancing_Ball.o: ../Src/App/Balancing_Ball.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F401RCTx -DDEBUG -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/App/Balancing_Ball.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

