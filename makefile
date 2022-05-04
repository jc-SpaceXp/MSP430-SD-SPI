# Makefile for ti-msp430
# intended for a single source (assembly) file (.asm)
# to be converted to a object (.o) and executable (.elf)

GCC_DIR = /opt/ti-msp430
SUPPORT_HEADERS = $(GCC_DIR)/lib/gcc/msp430-elf/9.3.1/include
SUPPORT_LINKS = $(GCC_DIR)/lib/gcc/msp430-elf/9.3.1/include

# Set your device here
MCU ?= msp430g2553
CC = msp430-elf-gcc
AS = msp430-elf-as
LD = msp430-elf-ld
GDB = msp430-elf-gdb

AFLAGS = -D --warn
LFLAGS = -L $(SUPPORT_LINKS) -m msp430elf -T $(MCU).ld

DEBUG ?= 0
ifeq ($(DEBUG), 1)
	AFLAGS += -g --gdwarf-2
endif

ENTRY_SECT ?= Reset

SRCS := $(wildcard *.asm)
OBJS := $(addsuffix .o,$(basename $(SRCS)))

TARGET := $(basename $(SRCS))

.PHONY: all
all: $(TARGET).elf

$(TARGET).elf: $(OBJS)
	@echo "--- Linking build"
	$(LD) $(LFLAGS) $(OBJS) -e $(ENTRY_SECT) -o $@

$(OBJS): $(SRCS)
	@echo "--- Compiling objects"
	$(AS) $(AFLAGS) -I $(SUPPORT_HEADERS) -mmcu=$(MCU) $(SRCS) -o $@


.PHONY: clean
clean:
	@echo "--- Cleaning build"
	-rm $(OBJS) $(TARGET).elf
