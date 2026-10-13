CXX      := g++

CXXFLAGS := -g --std=c++17 -Wall -Wextra
LDFLAGS  := -L../ManusSDK/lib -lManusSDK_Integrated -lGL -lGLU -lpthread -Wl,-rpath=../ManusSDK/lib

# GLFW和ImGui库
GLFW_LIBS := -lglfw
IMGUI_DIR := third_party/imgui

BUILD    := ./
OBJ_DIR  := $(BUILD)/objects
APP_DIR  := $(BUILD)
TARGET   := CalibrationGUI.out

INCLUDE  := -I../ManusSDK/include -I$(IMGUI_DIR) -I$(IMGUI_DIR)/backends

SRC      := \
   CalibrationApp.cpp \
   main.cpp \
   $(IMGUI_DIR)/imgui.cpp \
   $(IMGUI_DIR)/imgui_demo.cpp \
   $(IMGUI_DIR)/imgui_draw.cpp \
   $(IMGUI_DIR)/imgui_tables.cpp \
   $(IMGUI_DIR)/imgui_widgets.cpp \
   $(IMGUI_DIR)/backends/imgui_impl_glfw.cpp \
   $(IMGUI_DIR)/backends/imgui_impl_opengl3.cpp

OBJECTS  := $(SRC:%.cpp=$(OBJ_DIR)/%.o)
DEPENDENCIES := $(OBJECTS:.o=.d)

all: check_imgui build $(APP_DIR)/$(TARGET)

check_imgui:
	@if [ ! -d "$(IMGUI_DIR)" ]; then \
		echo "ImGui not found. Running download script..."; \
		./download_imgui.sh; \
	fi

$(OBJ_DIR)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $< -MMD -o $@

$(APP_DIR)/$(TARGET): $(OBJECTS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -o $(APP_DIR)/$(TARGET) $^ $(LDFLAGS) $(GLFW_LIBS)

-include $(DEPENDENCIES)

.PHONY: all build clean debug release info check_imgui

build:
	@mkdir -p $(APP_DIR)
	@mkdir -p $(OBJ_DIR)

debug: CXXFLAGS += -DDEBUG -g
debug: all

release: CXXFLAGS += -O2 -DNDEBUG
release: all

clean:
	-@rm -f $(TARGET)
	-@rm -rf $(OBJ_DIR)/*

info:
	@echo "[*] Application dir: ${APP_DIR}"
	@echo "[*] Object dir:      ${OBJ_DIR}"
	@echo "[*] Sources:         ${SRC}"
	@echo "[*] Objects:         ${OBJECTS}"
	@echo "[*] Dependencies:    ${DEPENDENCIES}"

