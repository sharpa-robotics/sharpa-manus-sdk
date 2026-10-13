#!/bin/bash

# 检查编译后的二进制文件是否依赖系统的libprotobuf

BINARY="SharpaManusClient.out"

if [ ! -f "$BINARY" ]; then
    echo "Error: Binary file $BINARY not found"
    echo "Please build the project first: make all"
    exit 1
fi

echo "=== Checking dependencies for $BINARY ==="
echo ""

# 使用ldd检查动态库依赖
echo "--- Dynamic library dependencies (ldd) ---"
ldd "$BINARY" | grep -i protobuf || echo "  ✓ No libprotobuf.so dependency found"

echo ""
echo "--- Checking for libprotobuf in binary (readelf) ---"
readelf -d "$BINARY" | grep -i protobuf || echo "  ✓ No libprotobuf reference found"

echo ""
echo "--- All dynamic library dependencies ---"
ldd "$BINARY"

echo ""
echo "=== Summary ==="
if ldd "$BINARY" | grep -qi "libprotobuf"; then
    echo "⚠️  WARNING: Binary depends on system libprotobuf.so"
    echo "   This may cause version conflicts!"
else
    echo "✓ Binary does NOT depend on system libprotobuf.so"
    echo "  Protobuf is statically linked ✓"
fi

