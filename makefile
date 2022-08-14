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

AFLAGS = -D --warn --strip-local-absolute
LFLAGS = -L $(SUPPORT_LINKS) --warn-section-align --warn-unresolved-symbols -q -m msp430elf -T $(MCU).ld

DEBUG ?= 0
ifeq ($(DEBUG), 1)
	AFLAGS += -g --gdwarf-2 --defsym DEBUG=1
endif

ENTRY_SECT ?= Reset

SRCS := $(wildcard *.asm)
OBJS := $(addsuffix .o,$(basename $(SRCS)))
PYTHON_SCRIPT := $(wildcard *.py)
PYTHON_SCRIPT_OUTPUT := $(wildcard *.inc)

TARGET := $(basename $(SRCS))

.PHONY: setup
setup: $(PYTHON_SCRIPT)
	@echo "--- Generating GNU AS header file"
	python3 $(PYTHON_SCRIPT) -d $(MCU) -I $(SUPPORT_HEADERS) -L $(SUPPORT_LINKS) --msp430_comments

.PHONY: all
all: setup $(TARGET).elf

$(TARGET).elf: $(OBJS)
	@echo "--- Linking build"
	$(LD) $(LFLAGS) $(OBJS) -e $(ENTRY_SECT) -o $@

$(OBJS): $(SRCS)
	@echo "--- Compiling objects"
	$(AS) $(AFLAGS) -I $(SUPPORT_HEADERS) -mmcu=$(MCU) $(SRCS) -o $@


.PHONY: clean
clean:
	@echo "--- Cleaning build"
	-rm $(OBJS) $(TARGET).elf $(PYTHON_SCRIPT_OUTPUT)
