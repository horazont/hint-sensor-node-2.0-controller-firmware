STM32SPL ?= ../../../../stm32/stm32_spl/

CC      = arm-none-eabi-gcc
CXX     = arm-none-eabi-g++
LD      = arm-none-eabi-ld
CP      = arm-none-eabi-objcopy
OD      = arm-none-eabi-objdump
SIZE    = arm-none-eabi-size

PLATFORM = STM32F10X_MD
STM_STDPERIPH_DEBUG = 1

LFLAGS  = -Tstm32_flash.ld -nostartfiles -Wl,--gc-sections -L/usr/lib/arm-none-eabi/newlib/thumb/ -lstdc++ -lc
CPFLAGS = -Obinary
ODFLAGS = -S

c_sources = $(wildcard src/*.c)
cxx_sources = $(filter-out lightsensor_freq.c,$(filter-out comm_xbee.cc,$(wildcard src/*.cc)))
headers = $(wildcard include/sbx/*.h)
obj_dir = .objs/
c_objs = $(patsubst src/%.c,$(obj_dir)%.o,$(c_sources))
cxx_objs = $(patsubst src/%.cc,$(obj_dir)%.o,$(cxx_sources))
objs = $(c_objs) $(cxx_objs)

startup_file_src = src/startup_stm32f10x_md.s
startup_file_obj = $(patsubst src/%.s,$(obj_dir)%.o,$(startup_file_src))

all: test

CFLAGS +=  -Wall -Wextra -Werror -I./include/sbx/ -ggdb -mcpu=cortex-m3 -mthumb -nostartfiles -fwrapv -static -Os -Wno-error=strict-aliasing -flto

include $(STM32SPL)/ST_STM32StdPeriph.mk

CXXFLAGS := -std=c++11 -fno-exceptions -fno-rtti $(CFLAGS)
CFLAGS := -std=c11 $(CFLAGS)

system_obj = $($(PFX)_PFP)obj/CMSIS/Device/ST/STM32F10x/Source/system_stm32f10x.o

all: main.elf

clean: vendor_st_stm32_stdperiph_clean
	rm -rf main.lst main.elf main.lst main.bin .objs

test: main.elf
	@ echo "...copying"
	$(CP) $(CPFLAGS) main.elf main.bin
	$(SIZE) main.elf


main.elf: $(objs) $(startup_file_obj) stm32_flash.ld $(SPL_LIB) $(system_obj)
	@ echo "..linking"
	$(CXX) $(LFLAGS) -o main.elf $(startup_file_obj) $(objs) $(SPL_LIB) $(system_obj)

# main.elf: $(objs) $(startup_file_obj) stm32_flash.ld $(SPL_LIB) $(system_obj)
# 	$(CC) $(CFLAGS) -Wl,-v -Wl,-Tstm32_flash.ld -L/usr/lib/arm-none-eabi/newlib/libc.a -Wl,-nostartfiles -Wl,--gc-sections -o main.elf $(startup_file_obj) $(objs) $(SPL_LIB) $(system_obj)

$(c_objs): $(obj_dir)%.o: src/%.c $(headers)
ifdef obj_dir
	@mkdir -p $(obj_dir)
endif
	$(CC) $(CFLAGS) -c -o $@ $<

$(cxx_objs): $(obj_dir)%.o: src/%.cc $(headers)
ifdef obj_dir
	@mkdir -p $(obj_dir)
endif
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(startup_file_obj): $(obj_dir)%.o: src/%.s
ifdef obj_dir
	@mkdir -p $(obj_dir)
endif
	$(CC) $(CFLAGS) -c -o $@ $<
