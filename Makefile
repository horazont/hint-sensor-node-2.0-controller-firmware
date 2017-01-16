CC      = arm-none-eabi-gcc
CXX     = arm-none-eabi-g++
LD      = arm-none-eabi-g++
CP      = arm-none-eabi-objcopy
OD      = arm-none-eabi-objdump

PLATFORM = STM32F10X_MD
STM_STDPERIPH_DEBUG = 1

LFLAGS  = -Tstm32_flash.ld  -L/usr/lib/arm-none-eabi/newlib/libc.a -nostartfiles -Wl,--gc-sections
CPFLAGS = -Obinary
ODFLAGS = -S

c_sources = $(wildcard *.c)
cxx_sources = $(wildcard *.cc)
headers = $(wildcard *.h)
c_objs = $(patsubst %.c,%.o,$(c_sources))
cxx_objs = $(patsubst %.cc,%.o,$(cxx_sources))
objs = $(c_objs) $(cxx_objs)

startup_file_src = startup_stm32f10x_md.s
startup_file_obj = $(patsubst %.s,%.o,$(startup_file_src))

all: test

include ../stm32_spl/ST_STM32StdPeriph.mk

CXXFLAGS := -std=c++11 -Wall -Wextra -Werror -I./ -fno-common -O0 -ggdb -mcpu=cortex-m3 -mthumb -nostartfiles -fno-exceptions $(CFLAGS)
CFLAGS := -std=c11 -Wall -Wextra -Werror -I./ -fno-common -O0 -ggdb -mcpu=cortex-m3 -mthumb -nostartfiles $(CFLAGS)

system_obj = $($(PFX)_PFP)obj/CMSIS/Device/ST/STM32F10x/Source/system_stm32f10x.o

clean: vendor_st_stm32_stdperiph_clean
	-rm -f main.lst main.elf main.lst main.bin $(objs) $(startup_file_obj)

test: main.elf
	@ echo "...copying"
	$(CP) $(CPFLAGS) main.elf main.bin
	$(OD) $(ODFLAGS) main.elf > main.lst


main.elf: $(objs) $(startup_file_obj) stm32_flash.ld $(SPL_LIB) $(system_obj)
	@ echo "..linking"
	$(LD) $(LFLAGS) -o main.elf $(startup_file_obj) $(objs) $(SPL_LIB) $(system_obj)

# main.elf: $(objs) $(startup_file_obj) stm32_flash.ld $(SPL_LIB) $(system_obj)
# 	$(CC) $(CFLAGS) -Wl,-v -Wl,-Tstm32_flash.ld -L/usr/lib/arm-none-eabi/newlib/libc.a -Wl,-nostartfiles -Wl,--gc-sections -o main.elf $(startup_file_obj) $(objs) $(SPL_LIB) $(system_obj)

$(c_objs): %.o: %.c $(headers)
	$(CC) $(CFLAGS) -c -o $@ $<

$(cxx_objs): %.o: %.cc $(headers)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(startup_file_obj): %.o: %.s
	$(CC) $(CFLAGS) -c -o $@ $<
