# Voyager FDS Emulator Makefile
# JPL Power of 10 compliant C

CC = gcc
CFLAGS = -Wall -Wextra -Werror -pedantic -std=c11 -O2
CFLAGS += -Wno-unused-parameter  # For now, during development

# Debug build
ifdef DEBUG
CFLAGS += -g -O0 -DDEBUG
endif

# Directories
SRC_DIR = src
INC_DIR = include
BUILD_DIR = build

# Source files
SRCS = $(SRC_DIR)/fds_cpu.c $(SRC_DIR)/fds_asm.c $(SRC_DIR)/main.c
OBJS = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRCS))

# Output
TARGET = fds

# Windows executable extension
ifeq ($(OS),Windows_NT)
TARGET := $(TARGET).exe
endif

.PHONY: all clean test

all: $(BUILD_DIR) $(TARGET)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c $(INC_DIR)/fds.h
	$(CC) $(CFLAGS) -I$(INC_DIR) -c -o $@ $<

test: $(TARGET)
	./$(TARGET) --test --debug

clean:
	rm -rf $(BUILD_DIR) $(TARGET)

# Run with step mode
step: $(TARGET)
	./$(TARGET) --test --step

# Disassemble a binary file
disasm: $(TARGET)
	@echo "Usage: ./$(TARGET) <file.bin>"
