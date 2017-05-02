################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Compiler'
	"/home/trandi/ti/ccsv6/tools/compiler/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc" -c -mcpu=cortex-m3 -mthumb -DPART_LM3S8962 -I"/home/trandi/ti/ccsv6/tools/compiler/gcc-arm-none-eabi-4_9-2015q3/arm-none-eabi/include" -I"/media/trandi/SSD_DATA/data/alex/ccs6-2_workspace_linux/LM3S8962-FreeRTOS-CAN" -I"/media/trandi/SSD_DATA/progsDan/TI/StellarisWare" -I"/media/trandi/SSD_DATA/progsDan/TI/StellarisWare/third_party/FreeRTOS/Source" -I"/media/trandi/SSD_DATA/progsDan/TI/StellarisWare/third_party/FreeRTOS/Source/portable/GCC/ARM_CM3" -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -Wall -specs="nosys.specs" -MD -std=c99 -pedantic -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o"$@" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '


