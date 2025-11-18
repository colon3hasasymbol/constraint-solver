# Makefile for compiling a C program with GLFW, yes this was made by chatgpt im sorry nonexistent chat

# Compiler and flags
CC = gcc
CFLAGS = -std=c99 -Iinc -march=native -g
LDFLAGS = -march=native -g -lraylib -lm

# Directories
SRC_DIR = src
OBJ_DIR = obj
INC_DIR = include

# Program name
TARGET = program

# Collect all C source files in the src directory
SRCS = $(wildcard $(SRC_DIR)/*.c)
# Generate object file names by replacing the src directory with the obj directory and changing .c to .o
OBJS = $(patsubst $(SRC_DIR)/%.c, $(OBJ_DIR)/%.o, $(SRCS))

# Default target
all: $(TARGET)

# Rule to build the target program
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# Rule to compile source files into object files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) -c -o $@ $<

# Run the program after compiling
run: $(TARGET)
	./$(TARGET)

# Clean up build files
clean:
	rm -rf $(OBJ_DIR) $(TARGET)

# Phony targets
.PHONY: all clean run
