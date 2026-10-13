#!/bin/bash

# 下载并编译protobuf 3.12.4
PROTOBUF_VERSION="3.12.4"
PROTOBUF_DIR="third_party/protobuf"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$SCRIPT_DIR"

# 检查是否已经存在且已编译
if [ -d "$PROTOBUF_DIR" ] && [ -f "$PROTOBUF_DIR/src/.libs/libprotobuf.a" ] && [ -f "$PROTOBUF_DIR/src/protoc" ]; then
    echo "Protobuf already downloaded and built"
    exit 0
fi

echo "Downloading protobuf v${PROTOBUF_VERSION}..."

# 清理旧目录
rm -rf "$PROTOBUF_DIR"
mkdir -p "$PROTOBUF_DIR"

# 下载protobuf源码
TMP_FILE="/tmp/protobuf_v${PROTOBUF_VERSION}.tar.gz"
PROTOBUF_URL="https://github.com/protocolbuffers/protobuf/releases/download/v${PROTOBUF_VERSION}/protobuf-cpp-${PROTOBUF_VERSION}.tar.gz"

if command -v wget &> /dev/null; then
    echo "Using wget to download..."
    wget "$PROTOBUF_URL" -O "$TMP_FILE"
elif command -v curl &> /dev/null; then
    echo "Using curl to download..."
    curl -L "$PROTOBUF_URL" -o "$TMP_FILE"
else
    echo "Error: wget or curl is required"
    exit 1
fi

# 检查下载是否成功
if [ ! -f "$TMP_FILE" ]; then
    echo "Error: Failed to download protobuf"
    exit 1
fi

# 解压
echo "Extracting protobuf..."
tar -xzf "$TMP_FILE" -C "$PROTOBUF_DIR" --strip-components=1

if [ ! -f "$PROTOBUF_DIR/configure" ]; then
    echo "Error: Failed to extract protobuf"
    rm -f "$TMP_FILE"
    exit 1
fi

rm -f "$TMP_FILE"

# 编译protobuf
echo "Configuring protobuf..."
cd "$PROTOBUF_DIR"
./configure --prefix="$SCRIPT_DIR/$PROTOBUF_DIR/install" --disable-shared --enable-static

if [ $? -ne 0 ]; then
    echo "Error: Failed to configure protobuf"
    exit 1
fi

echo "Building protobuf (this may take a few minutes)..."
make -j$(nproc)

if [ $? -ne 0 ]; then
    echo "Error: Failed to build protobuf"
    exit 1
fi

# 检查编译结果
if [ ! -f "src/.libs/libprotobuf.a" ] || [ ! -f "src/protoc" ]; then
    echo "Error: Protobuf build incomplete"
    exit 1
fi

echo "Protobuf v${PROTOBUF_VERSION} downloaded and built successfully"
echo "Static library: $PROTOBUF_DIR/src/.libs/libprotobuf.a"
echo "Protoc compiler: $PROTOBUF_DIR/src/protoc"

