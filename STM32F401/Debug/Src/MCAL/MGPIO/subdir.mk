################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/MCAL/MGPIO/MGPIO_Prog.c 

OBJS += \
./Src/MCAL/MGPIO/MGPIO_Prog.o 

C_DEPS += \
./Src/MCAL/MGPIO/MGPIO_Prog.d 


# Each subdirectory must supply rules for building sources it contributes
Src/MCAL/MGPIO/MGPIO_Prog.o: ../Src/MCAL/MGPIO/MGPIO_Prog.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F401RCTx -DDEBUG -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/MCAL/MGPIO/MGPIO_Prog.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

