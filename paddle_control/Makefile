# QNX aarch64 Configuration
QNX_HOST ?= /opt/qnx800/host/linux/x86_64
QNX_TARGET ?= /opt/qnx800/target/qnx8

# Compiler and flags for QNX aarch64
CC := qcc -Vgcc_ntoaarch64le
CXX := qcc -Vgcc_ntoaarch64le_cxx
CFLAGS := -Wall -Wextra -O2
CXXFLAGS := -Wall -Wextra -O2
LDFLAGS := 

# QNX-specific flags
QNX_CFLAGS := -D_QNX_SOURCE -fpermissive
QNX_LDFLAGS := 

# Directories
SRC_DIR := src
INC_DIR := include
BUILD_DIR := build
BIN_DIR := bin

# Target executable name
TARGET := $(BIN_DIR)/program

# Recursively find all source files
C_SRCS := $(shell find $(SRC_DIR) -name '*.c' 2>/dev/null)
CXX_SRCS := $(shell find $(SRC_DIR) -name '*.cpp' 2>/dev/null)

# Generate object file paths
C_OBJS := $(C_SRCS:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.o)
CXX_OBJS := $(CXX_SRCS:$(SRC_DIR)/%.cpp=$(BUILD_DIR)/%.o)
OBJS := $(C_OBJS) $(CXX_OBJS)

# Recursively find all header directories
INC_DIRS := $(shell find $(INC_DIR) -type d 2>/dev/null)
INC_FLAGS := $(addprefix -I,$(INC_DIRS))

# Add QNX include paths
INC_FLAGS += -I$(QNX_TARGET)/usr/include

# Dependency files
DEPS := $(OBJS:.o=.d)

# Combine flags
ALL_CFLAGS := $(CFLAGS) $(QNX_CFLAGS) $(INC_FLAGS)
ALL_CXXFLAGS := $(CXXFLAGS) $(QNX_CFLAGS) $(INC_FLAGS)
ALL_LDFLAGS := $(LDFLAGS) $(QNX_LDFLAGS)

# Default target
.PHONY: all
all: $(TARGET)

# Link object files to create executable
$(TARGET): $(OBJS)
	@mkdir -p $(dir $@)
	$(CXX) $(OBJS) -o $@ $(ALL_LDFLAGS)
	@echo "Build complete: $(TARGET)"

# Compile C source files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(ALL_CFLAGS) -MMD -MP -c $< -o $@

# Compile C++ source files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(ALL_CXXFLAGS) -MMD -MP -c $< -o $@

# Include dependency files
-include $(DEPS)

# Clean build artifacts
.PHONY: clean
clean:
	rm -rf $(BUILD_DIR) $(BIN_DIR)
	@echo "Clean complete"

# Deploy to QNX target (adjust IP/path as needed)
.PHONY: deploy
deploy: $(TARGET)
	@echo "Deploying to QNX target..."
	scp $(TARGET) root@qnx-target:/tmp/
	@echo "Deploy complete"

# Run the program
.PHONY: run
run: $(TARGET)
	./$(TARGET)

# Print variables for debugging
.PHONY: print
print:
	@echo "QNX_HOST: $(QNX_HOST)"
	@echo "QNX_TARGET: $(QNX_TARGET)"
	@echo "CC: $(CC)"
	@echo "CXX: $(CXX)"
	@echo "C Sources: $(C_SRCS)"
	@echo "C++ Sources: $(CXX_SRCS)"
	@echo "Objects: $(OBJS)"
	@echo "Include Dirs: $(INC_DIRS)"
	@echo "CFLAGS: $(ALL_CFLAGS)"

# Create directory structure
.PHONY: init
init:
	mkdir -p $(SRC_DIR) $(INC_DIR) $(BUILD_DIR) $(BIN_DIR)
	@echo "Project directories created"

# Check QNX environment
.PHONY: check-env
check-env:
	@echo "Checking QNX environment..."
	@if [ -z "$(QNX_HOST)" ]; then echo "ERROR: QNX_HOST not set"; exit 1; fi
	@if [ -z "$(QNX_TARGET)" ]; then echo "ERROR: QNX_TARGET not set"; exit 1; fi
	@which qcc > /dev/null || (echo "ERROR: qcc not found in PATH"; exit 1)
	@echo "QNX environment OK"
