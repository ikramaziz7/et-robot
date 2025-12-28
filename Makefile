CC          = arm-linux-gnueabi-gcc
CFLAGS      = -O2 -g -std=gnu99 -W -Wall -Wno-comment
INCLUDES    = -I./ev3dev-c/source/ev3 -I./include/
LDFLAGS     = -L./libraries -lrt -lm -lev3dev-c -lpthread
BUILD_DIR   = ./build
SOURCE_DIR  = ./src

OBJS = \
	$(BUILD_DIR)/movement.o \
	$(BUILD_DIR)/position.o \
	$(BUILD_DIR)/sensors.o \
	$(BUILD_DIR)/state_machine.o \
	$(BUILD_DIR)/time_utils.o \
	$(BUILD_DIR)/main.o

export LD_LIBRARY_PATH := $(CURDIR)/ev3dev-c/lib

all: main

main: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) -o main

# Create build directory if missing
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Compile each source file to object file
$(BUILD_DIR)/%.o: $(SOURCE_DIR)/%.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $(INCLUDES) $< -o $@

clean:
	rm -rf $(BUILD_DIR)
	rm -f ./main