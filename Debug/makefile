################################################################################
# Automatically-generated file. Do not edit!
################################################################################

CG_TOOL_ROOT := /home/trandi/ti/ccsv6/tools/compiler/gcc-arm-none-eabi-4_9-2015q3

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./lm3s8962_startup_ccs_gcc.o" \
"./main.o" \
"./rit128x96x4.o" \
"./ustdlib.o" \
"./freeRTOS/heap_1.o" \
"./freeRTOS/list.o" \
"./freeRTOS/port.o" \
"./freeRTOS/queue.o" \
"./freeRTOS/tasks.o" \
-Wl,-T"../lm3s8962.lds" \
$(GEN_CMDS__FLAG) \
-Wl,--start-group \
-ldriver-cm3 \
-lc \
-Wl,--end-group \

-include ../makefile.init

RM := rm -rf
RMDIR := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include freeRTOS/subdir_vars.mk
-include subdir_rules.mk
-include freeRTOS/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
LM3S8962-FreeRTOS-CAN.out \

EXE_OUTPUTS__QUOTED += \
"LM3S8962-FreeRTOS-CAN.out" \

BIN_OUTPUTS += \
LM3S8962-FreeRTOS-CAN.hex \

BIN_OUTPUTS__QUOTED += \
"LM3S8962-FreeRTOS-CAN.hex" \


# All Target
all: LM3S8962-FreeRTOS-CAN.out

# Tool invocations
LM3S8962-FreeRTOS-CAN.out: $(OBJS) $(CMD_SRCS) $(LDS_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU Linker'
	"/home/trandi/ti/ccsv6/tools/compiler/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc" -mcpu=cortex-m3 -mthumb -DPART_LM3S8962 -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -Wall -specs="nosys.specs" -Wl,-Map,"LM3S8962-FreeRTOS-CAN.map" -L"/media/trandi/SSD_DATA/progsDan/TI/StellarisWare/driverlib/gcc-cm3" -o"LM3S8962-FreeRTOS-CAN.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

LM3S8962-FreeRTOS-CAN.hex: $(EXE_OUTPUTS)
	@echo 'Invoking: GNU Objcopy Utility'
	"/home/trandi/ti/ccsv6/tools/compiler/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-objcopy"  $(EXE_OUTPUTS__QUOTED) "LM3S8962-FreeRTOS-CAN.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)$(BIN_OUTPUTS__QUOTED)
	-$(RM) "lm3s8962_startup_ccs_gcc.d" "main.d" "rit128x96x4.d" "ustdlib.d" "freeRTOS/heap_1.d" "freeRTOS/list.d" "freeRTOS/port.d" "freeRTOS/queue.d" "freeRTOS/tasks.d" 
	-$(RM) "lm3s8962_startup_ccs_gcc.o" "main.o" "rit128x96x4.o" "ustdlib.o" "freeRTOS/heap_1.o" "freeRTOS/list.o" "freeRTOS/port.o" "freeRTOS/queue.o" "freeRTOS/tasks.o" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

