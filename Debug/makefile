################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
MukkeBox_STM32F030C6.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
MukkeBox_STM32F030C6.list \

OBJCOPY_BIN += \
MukkeBox_STM32F030C6.bin \


# All Target
all: MukkeBox_STM32F030C6.elf secondary-outputs

# Tool invocations
MukkeBox_STM32F030C6.elf: $(OBJS) $(USER_OBJS) /home/niko/STM32CubeIDE/workspace_1.3.0/MukkeBox_STM32F030C6/STM32F030C6TX_FLASH.ld
	arm-none-eabi-gcc -o "MukkeBox_STM32F030C6.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"/home/niko/STM32CubeIDE/workspace_1.3.0/MukkeBox_STM32F030C6/STM32F030C6TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="MukkeBox_STM32F030C6.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

MukkeBox_STM32F030C6.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "MukkeBox_STM32F030C6.list"
	@echo 'Finished building: $@'
	@echo ' '

MukkeBox_STM32F030C6.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "MukkeBox_STM32F030C6.bin"
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
