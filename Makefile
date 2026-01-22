FILENAME=kernel
SRC_DIR:=src
INCLUDE_DIR:=include
BUILD_DIR:=build
XDIR:=/u/cs452/public/xdev
TRIPLE=aarch64-none-elf
XBINDIR:=$(XDIR)/bin
CC:=$(XBINDIR)/$(TRIPLE)-gcc -ffreestanding
OBJCOPY:=$(XBINDIR)/$(TRIPLE)-objcopy
OBJDUMP:=$(XBINDIR)/$(TRIPLE)-objdump

OPT?=-O3

WARNINGS:=-Wall -Wextra -Wpedantic -Wno-unused-const-variable
CFLAGS:=-O3 -g -pipe -static -march=armv8-a -mcpu=cortex-a72 $(OPT) -mstrict-align -mgeneral-regs-only $(WARNINGS) -I$(INCLUDE_DIR)

# -Wl,option tells gcc to pass 'option' to the linker with commas replaced by spaces
LDFLAGS:=-Wl,-nmagic -Wl,-Tlinker.ld -Wl,--no-warn-rwx-segments -nostartfiles

# Source files
SRCS := $(shell find $(SRC_DIR) -name '*.c')
ASM_SRCS := $(shell find $(SRC_DIR) -name '*.S')
SOURCES := $(SRCS) $(ASM_SRCS)
OBJECTS := $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRCS))
OBJECTS += $(patsubst $(SRC_DIR)/%.S,$(BUILD_DIR)/%.o,$(ASM_SRCS))
DEPENDS := $(OBJECTS:.o=.d)

.PHONY: all clean

all: $(BUILD_DIR)/$(FILENAME).img

clean:
	rm -rf $(BUILD_DIR)

$(BUILD_DIR)/$(FILENAME).img: $(BUILD_DIR)/$(FILENAME).elf
	$(OBJCOPY) -S -O binary $< $@; sync

$(BUILD_DIR)/$(FILENAME).elf: $(OBJECTS) linker.ld
	$(CC) $(CFLAGS) $(filter-out %.ld, $^) -o $@ $(LDFLAGS)
	@$(OBJDUMP) -d $@ | grep -Fq q0 && printf "\n***** WARNING: SIMD DETECTED! *****\n\n" || true

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.S Makefile
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c Makefile
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

-include $(DEPENDS)