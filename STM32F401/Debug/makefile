################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/MCAL/MUART/subdir.mk
-include Src/MCAL/MTIMER/subdir.mk
-include Src/MCAL/MSTK/subdir.mk
-include Src/MCAL/MSPI/subdir.mk
-include Src/MCAL/MRCC/subdir.mk
-include Src/MCAL/MNVIC/subdir.mk
-include Src/MCAL/MIWDT/subdir.mk
-include Src/MCAL/MI2C/subdir.mk
-include Src/MCAL/MGPIO/subdir.mk
-include Src/MCAL/MFPU/subdir.mk
-include Src/MCAL/MEXTI/subdir.mk
-include Src/HAL/HULTRA_SONIC/subdir.mk
-include Src/HAL/HSERVO/subdir.mk
-include Src/HAL/HLCD/subdir.mk
-include Src/App/subdir.mk
-include Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Balancing\ Ball.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Balancing\ Ball.list \

OBJCOPY_BIN += \
Balancing\ Ball.bin \


# All Target
all: Balancing\ Ball.elf secondary-outputs

# Tool invocations
Balancing\ Ball.elf: $(OBJS) $(USER_OBJS) G:\Embedded\ Systems\Projects\Balancing\ Ball\STM32F401RCTX_FLASH.ld
	arm-none-eabi-gcc -o "Balancing Ball.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"G:\Embedded Systems\Projects\Balancing Ball\STM32F401RCTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Balancing Ball.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Balancing\ Ball.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Balancing Ball.list"
	@echo 'Finished building: $@'
	@echo ' '

Balancing\ Ball.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Balancing Ball.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
