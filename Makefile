# Makefile for compiling C projects with Raylib on Windows

# Path to Raylib (replace with the correct path)
RAYLIB_PATH = C:/Code/c/lib/raylib

# Compiler and Flags
CC = gcc
CFLAGS = -I$(RAYLIB_PATH)/include -I./src
LDFLAGS = -L$(RAYLIB_PATH)/lib
LIBS = -lraylib -lpthread -lopengl32 -lgdi32 -lwinmm -static

# Executable Name
TARGET = pid_sim

# Source Files
SRC = src/main.c src/pid.c src/sim.c src/graph.c src/utils.c

# Object Files
OBJ = $(SRC:.c=.o)

# Default Rule
all: $(TARGET)

# Link Object Files to Create Executable
$(TARGET): $(OBJ)
	@echo "Compiling the project..."
	$(CC) -o $(TARGET).exe $(OBJ) $(CFLAGS) $(LDFLAGS) $(LIBS)
	@echo "Build completed! Generated executable: $(TARGET).exe"

# Compile .c files to .o object files
%.o: %.c
	@echo "Compiling $<..."
	$(CC) -c $< -o $@ $(CFLAGS)

# Clean Generated Files
clean:
	@echo "Cleaning files..."
	del $(TARGET).exe $(OBJ)
	@echo "Cleaning completed!"

# Run the Executable
run: $(TARGET)
	@echo "Running the project..."
	./$(TARGET).exe

# Rebuild the Executable
rebuild: clean all

# Phony Targets
.PHONY: all clean run rebuild
