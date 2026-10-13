CXX      := g++

#CXXFLAGS := -pedantic-errors -Wall -Wextra -Werror -g --std=c++17

# Protobuf配置 - 使用本地编译的版本
PROTOBUF_DIR := third_party/protobuf
PROTOC := $(PROTOBUF_DIR)/src/protoc
PROTOBUF_INCLUDE := -I$(PROTOBUF_DIR)/src
PROTOBUF_LIB := $(PROTOBUF_DIR)/src/.libs/libprotobuf.a

# 检查protobuf是否存在，如果不存在则下载并编译
check_protobuf:
	@if [ ! -d "$(PROTOBUF_DIR)" ] || [ ! -f "$(PROTOBUF_LIB)" ] || [ ! -f "$(PROTOC)" ]; then \
		echo "Protobuf not found. Running download script..."; \
		./download_protobuf.sh; \
	fi

CXXFLAGS := -g --std=c++17
CXXFLAGS += $(PROTOBUF_INCLUDE)
# 使用静态链接，避免依赖系统的libprotobuf
LDFLAGS  := -L./ManusSDK/lib -lManusSDK_Integrated -lncurses -lzmq -Wl,-rpath=./ManusSDK/lib
# 静态链接protobuf，使用-Wl,-Bstatic强制静态链接
LDFLAGS  += -Wl,-Bstatic $(PROTOBUF_LIB) -Wl,-Bdynamic -pthread

BUILD    := ./
OBJ_DIR  := $(BUILD)/objects
APP_DIR  := $(BUILD)
TARGET   := SharpaManusClient.out
INCLUDE  := -I./ManusSDK/include -I/usr/include/eigen3
INCLUDE  += $(PROTOBUF_INCLUDE)

SRC      :=  \
   $(wildcard *.cpp) \
   $(wildcard *.cc)

OBJECTS  := $(patsubst %.cpp,$(OBJ_DIR)/%.o,$(filter %.cpp,$(SRC))) \
            $(patsubst %.cc,$(OBJ_DIR)/%.o,$(filter %.cc,$(SRC)))
DEPENDENCIES \
         := $(OBJECTS:.o=.d)

# 编译.proto文件
%.pb.cc %.pb.h: %.proto
	@echo "Compiling $<..."
	@$(PROTOC) --cpp_out=. $<

all: check_protobuf build $(APP_DIR)/$(TARGET)

$(OBJ_DIR)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $< -MMD -o $@

$(OBJ_DIR)/%.o: %.cc
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $< -MMD -o $@

$(APP_DIR)/$(TARGET): $(OBJECTS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -o $(APP_DIR)/$(TARGET) $^ $(LDFLAGS)
	
-include $(DEPENDENCIES)

.PHONY: all build clean debug release info check_protobuf

build:
	@mkdir -p $(APP_DIR)
	@mkdir -p $(OBJ_DIR)

debug: CXXFLAGS += -DDEBUG -g
debug: all

release: CXXFLAGS += -g
release: all

clean:
	-@rm $(TARGET)
	-@rm -rvf $(OBJ_DIR)/*

show-protobuf:
	@echo "---- Protobuf include path ----"
	@echo "$(PROTOBUF_INCLUDE)"
	@echo "---- Protobuf library path ----"
	@echo "$(PROTOBUF_LIB)"
	@echo "---- Protoc compiler ----"
	@echo "$(PROTOC)"
	@if [ -f "$(PROTOC)" ]; then \
		echo "---- Protoc version ----"; \
		$(PROTOC) --version; \
	fi

info:
	@echo "[*] Application dir: ${APP_DIR}     "
	@echo "[*] Object dir:      ${OBJ_DIR}     "
	@echo "[*] Sources:         ${SRC}         "
	@echo "[*] Objects:         ${OBJECTS}     "
	@echo "[*] Dependencies:    ${DEPENDENCIES}"
	@echo "[*] Protobuf dir:    ${PROTOBUF_DIR}"
	@echo "[*] Protoc:          ${PROTOC}"

