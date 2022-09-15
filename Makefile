all:
	arm-none-eabi-gcc -mcpu=cortex-m4 -c -std=gnu11 main.c -o main.o
	arm-none-eabi-gcc -mcpu=cortex-m4 -c -x assembler-with-cpp startup_stm32f411vetx.s -o start.o
	arm-none-eabi-gcc -mcpu=cortex-m4 main.o start.o -o prj.elf -T"STM32F411VETX_FLASH.ld" -Wl,-Map="prj.map" -Wl,--gc-sections -static
	arm-none-eabi-objcopy -O binary prj.elf app.bin
clean:
	-rm *.o *.elf *.map *.bin
