CC      = arm-none-eabi-gcc
LD      = arm-none-eabi-ld -v
CP      = arm-none-eabi-objcopy
OD      = arm-none-eabi-objdump

CFLAGS  = -std=c11 -Wall -Wextra -Werror -I./ -c -fno-common -O0 -g -mcpu=cortex-m3 -mthumb
LFLAGS  = -Tstm32.ld -nostartfiles
CPFLAGS = -Obinary
ODFLAGS = -S

all: test

clean:
	-rm -f main.lst main.o main.elf main.lst main.bin

test: blinky.elf
	@ echo "...copying"
	$(CP) $(CPFLAGS) main.elf main.bin
	$(OD) $(ODFLAGS) main.elf > main.lst

blinky.elf: main.o stm32.ld
	@ echo "..linking"
	$(LD) $(LFLAGS) -o main.elf main.o

main.o: main.c
	@ echo ".compiling"
	$(CC) $(CFLAGS) main.c
