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

WARNINGS:=-Wall -Wextra -Wpedantic -Wno-unused-const-variable
CFLAGS:=-g -pipe -static -march=armv8-a -mcpu=cortex-a72 -mstrict-align -mgeneral-regs-only $(WARNINGS) -I$(INCLUDE_DIR)

# -Wl,option tells gcc to pass 'option' to the linker with commas replaced by spaces
LDFLAGS:=-Wl,-nmagic -Wl,-Tlinker.ld -Wl,--no-warn-rwx-segments -nostartfiles


# ========== Compile Options ================

OPT?=1
CACHE?=b
VERBOSE?=0

MAKESPEC:=.make_spec
MAKESPEC_FORMAT:=$(VERBOSE) $(OPT) $(CACHE)

# clean up the built files, if the passed arguments have changed
ifneq ($(shell cat $(MAKESPEC) 2>/dev/null), $(MAKESPEC_FORMAT))
$(shell rm -rf $(BUILD_DIR))
$(shell echo $(MAKESPEC_FORMAT) > $(MAKESPEC))
endif


# VERBOSE (0, 1): Whether to print more verbose debug flags
ifeq ($(VERBOSE),1)
	CFLAGS += -DVERBOSE
endif

# OPT (0, 1): Whether to use O3 optimization
ifeq ($(OPT),1)
	CFLAGS += -O3 -DOPT
endif

# CACHE (n, i, d, b): Whether to enable the data caches or instruction caches
ifeq ($(CACHE),b)
	CFLAGS += -DICACHE
	CFLAGS += -DDCACHE
else ifeq ($(CACHE),i)
	CFLAGS += -DICACHE
else ifeq ($(CACHE),d)
	CFLAGS += -DDCACHE
endif

# =========================================

# Source files
SRCS := $(shell find $(SRC_DIR) -name '*.c')
ASM_SRCS := $(shell find $(SRC_DIR) -name '*.S')
SOURCES := $(SRCS) $(ASM_SRCS)
OBJECTS := $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRCS))
OBJECTS += $(patsubst $(SRC_DIR)/%.S,$(BUILD_DIR)/%.o,$(ASM_SRCS))
DEPENDS := $(OBJECTS:.o=.d)

.PHONY: all clean sim sim-debug sim-gui

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


# =========================================
# Simulate the raspberry pi on QEMU

QEMU = qemu-system-aarch64
QEMU_FLAGS = -M raspi4b -kernel $(BUILD_DIR)/$(FILENAME).img -nographic


sim: $(BUILD_DIR)/$(FILENAME).img
	$(QEMU) $(QEMU_FLAGS)

sim-debug: $(BUILD_DIR)/$(FILENAME).img
	$(QEMU) $(QEMU_FLAGS) -s -S

sim-gui: $(BUILD_DIR)/$(FILENAME).img
	$(QEMU) -M raspi4b -kernel $(BUILD_DIR)/$(FILENAME).img -serial stdio

# =========================================

-include $(DEPENDS)
