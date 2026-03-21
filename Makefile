# Makefile for the STM32F446RE Nucleo Board

# Toolchain and tools
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size
OPENOCD = openocd

# Directories
STARTUP_DIR 	= Startup
CMSIS_DIR   	= CMSIS
SRC_DIR         = Core/Src
INC_DIR         = Core/Inc
PERIPHERALS_DIR = Drivers/Peripherals
HARDWARE_DIR 	= Drivers/Hardware
BUILD_DIR       = build

# Source files
C_SOURCES = \
    $(wildcard $(SRC_DIR)/*.c) \
    $(wildcard $(PERIPHERALS_DIR)/*.c) \
    $(wildcard $(HARDWARE_DIR)/*.c) \

ASM_SOURCES = $(STARTUP_DIR)/startup_stm32f446re.s

# Output files
ELF_FILE = $(BUILD_DIR)/firmware.elf
BIN_FILE = $(BUILD_DIR)/firmware.bin

# Compiler flags
CFLAGS = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -g -O0 \
         -ffunction-sections -fdata-sections \
         -Wall -Wextra \
         -DSTM32F446xx \
         -MMD -MP \
         -I$(INC_DIR) \
         -I$(CMSIS_DIR) \
         -I$(STARTUP_DIR) \
         -I$(PERIPHERALS_DIR) \
         -I$(HARDWARE_DIR) \


LDFLAGS = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -T$(STARTUP_DIR)/STM32F446RETX_FLASH.ld \
          --specs=nosys.specs -lc -lm -lnosys \
          -Wl,-Map=$(BUILD_DIR)/firmware.map,--cref -Wl,--gc-sections

# Build object files from source files
C_OBJECTS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(notdir $(C_SOURCES)))
ASM_OBJECTS = $(patsubst %.s,$(BUILD_DIR)/%.o,$(notdir $(ASM_SOURCES)))
OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

# vpath for source files
vpath %.c $(SRC_DIR) $(STARTUP_DIR) $(PERIPHERALS_DIR) $(HARDWARE_DIR) 
vpath %.s $(STARTUP_DIR)

# Default target
all: $(BUILD_DIR) $(ELF_FILE) $(BIN_FILE)
	$(SIZE) $(ELF_FILE)

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Compile C source files
$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c -o $@ $<

# Compile assembly source files
$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c -o $@ $<

# Link object files to create the ELF file
$(ELF_FILE): $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

# Generate binary file from ELF file
$(BIN_FILE): $(ELF_FILE)
	$(OBJCOPY) -O binary $< $@

# Flash the microcontroller
flash: $(ELF_FILE)
	$(OPENOCD) -f interface/stlink.cfg -f target/stm32f4x.cfg \
	-c "program $(ELF_FILE) verify reset exit"

# Clean build artifacts
clean:
	rm -rf $(BUILD_DIR)

# Include generated dependency files
-include $(C_OBJECTS:.o=.d)

# Phony targets
.PHONY: all flash clean








