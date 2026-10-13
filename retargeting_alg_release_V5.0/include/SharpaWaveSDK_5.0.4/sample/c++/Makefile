# Makefile for Sharpa sample programs
# Tactile examples need OpenCV: sudo apt install libopencv-dev
#   (pkg-config opencv4 or opencv)

CXX = g++
CXXFLAGS = -std=c++17 -g -O0 -Wall -Wextra -DDEBUG -D_DEBUG
BUILD_DIR = build/

OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv)
OPENCV_LIBS := $(shell pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv)
ifeq ($(strip $(OPENCV_LIBS)),)
  $(error OpenCV not found via pkg-config. Install libopencv-dev, or set OPENCV_CFLAGS / OPENCV_LIBS manually)
endif

# SDK installed to /opt/sharpa-wave-sdk. For CI or local testing without dpkg -i, set SHARPA_SDK_ROOT to the
# unpacked deb staging root (e.g. dist/sharpa-wave-sdk_5.0.0_amd64/opt/sharpa-wave-sdk).
SHARPA_SDK_ROOT ?= /opt/sharpa-wave-sdk
INCLUDE_DIR = $(SHARPA_SDK_ROOT)/include
LIB_DIR = $(SHARPA_SDK_ROOT)/lib
LIBS = -lsharpa-wave-sdk -ldl

TACTILE_COMMON = tactile_sample_common.hpp
TARGET_FETCH = $(BUILD_DIR)/sharpa_tactile_fetch
SOURCE_FETCH = sharpa_tactile_fetch.cc
TARGET_CB = $(BUILD_DIR)/sharpa_tactile_callback
SOURCE_CB = sharpa_tactile_callback.cc
TARGET2 = $(BUILD_DIR)/sharpa_wave_example
SOURCE2 = sharpa_wave_example.cc

# Linker flags with RPATH to remember absolute library path
LDFLAGS = -Wl,--allow-multiple-definition -static-libgcc -static-libstdc++ -rdynamic -Wl,-rpath,$(LIB_DIR)

# Default target - build all examples
all: $(BUILD_DIR) $(TARGET_FETCH) $(TARGET_CB) $(TARGET2)

# Create build directory
$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

$(TARGET_FETCH): $(SOURCE_FETCH) $(TACTILE_COMMON) | $(BUILD_DIR)
	@echo "Building sharpa_tactile_fetch..."
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) $(SOURCE_FETCH) \
		-L$(LIB_DIR) $(LIBS) $(OPENCV_LIBS) \
		-I$(INCLUDE_DIR) \
		$(LDFLAGS) \
		-o $(TARGET_FETCH)
	@echo "✓ sharpa_tactile_fetch built"

$(TARGET_CB): $(SOURCE_CB) $(TACTILE_COMMON) | $(BUILD_DIR)
	@echo "Building sharpa_tactile_callback..."
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) $(SOURCE_CB) \
		-L$(LIB_DIR) $(LIBS) $(OPENCV_LIBS) \
		-I$(INCLUDE_DIR) \
		$(LDFLAGS) \
		-o $(TARGET_CB)
	@echo "✓ sharpa_tactile_callback built"

# Build sharpa_wave_example
$(TARGET2): $(SOURCE2) | $(BUILD_DIR)
	@echo "Building sharpa_wave_example..."
	@echo "  Using library path: $(LIB_DIR)"
	$(CXX) $(CXXFLAGS) $(SOURCE2) \
		-L$(LIB_DIR) $(LIBS) \
		-I$(INCLUDE_DIR) \
		$(LDFLAGS) \
		-o $(TARGET2)
	@echo "✓ sharpa_wave_example built successfully"
	@echo "  RPATH set to: $(LIB_DIR)"

# Build only tactile examples
tactile: $(BUILD_DIR) $(TARGET_FETCH) $(TARGET_CB)

# Build only wave example
wave: $(BUILD_DIR) $(TARGET2)

# Clean target
clean:
	@echo "Cleaning build directory..."
	@rm -f $(TARGET_FETCH) $(TARGET_CB) $(TARGET2)
	@rm -rf $(BUILD_DIR)
	@echo "✓ Clean complete"

# Help target
help:
	@echo "Sharpa Sample Makefile"
	@echo "Usage: make [target]"
	@echo ""
	@echo "Targets:"
	@echo "  all      Build all sample programs (default)"
	@echo "  tactile  Build sharpa_tactile_fetch + sharpa_tactile_callback"
	@echo "  wave     Build only sharpa_wave_example"
	@echo "  clean    Clean build directory"
	@echo "  help     Show this help message"
	@echo ""
	@echo "Requires: libopencv-dev for tactile examples."
	@echo ""
	@echo "SDK paths (/opt/sharpa-wave-sdk):"
	@echo "  Default: /opt/sharpa-wave-sdk/include and /opt/sharpa-wave-sdk/lib (after dpkg -i)."
	@echo "  Custom root: make SHARPA_SDK_ROOT=/path/to/sdk"
	@echo "    (INCLUDE_DIR/LIB_DIR become SHARPA_SDK_ROOT/include|lib; RPATH uses absolute LIB_DIR)."
	@echo ""
	@echo "Current configuration:"
	@echo "  Include Directory: $(INCLUDE_DIR)"
	@echo "  Library Directory: $(LIB_DIR)"
	@echo ""
	@echo "Note: Targets are built with RPATH to remember absolute library paths"

.PHONY: all tactile wave clean help
