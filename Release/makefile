################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Drivers/BSP/Components/lsm6dsl/subdir.mk
-include Drivers/BSP/Components/lps22hb/subdir.mk
-include Drivers/BSP/Components/lis3mdl/subdir.mk
-include Drivers/BSP/Components/hts221/subdir.mk
-include Drivers/BSP/Components/es_wifi/subdir.mk
-include Drivers/BSP/Components/M24SR/subdir.mk
-include Drivers/BSP/B-L475E-IOT01/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := Assignment 2 Integration
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Assignment\ 2\ Integration.elf \

MAP_FILES += \
Assignment\ 2\ Integration.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Assignment\ 2\ Integration.list \


# All Target
all: main-build

# Main-build Target
main-build: Assignment\ 2\ Integration.elf secondary-outputs

# Tool invocations
Assignment\ 2\ Integration.elf Assignment\ 2\ Integration.map: $(OBJS) $(USER_OBJS) C:\University\ Stuff\Y2S1\EE2028\EE2028_STM_CUBE_WS\Assignment\ 2\ Integration\STM32L475VGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Assignment 2 Integration.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\University Stuff\Y2S1\EE2028\EE2028_STM_CUBE_WS\Assignment 2 Integration\STM32L475VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Assignment 2 Integration.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Assignment\ 2\ Integration.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Assignment 2 Integration.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Assignment\ 2\ Integration.elf Assignment\ 2\ Integration.list Assignment\ 2\ Integration.map Assignment\\ 2\\ Integration.elf default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
