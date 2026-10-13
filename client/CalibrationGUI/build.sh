#!/bin/bash

# 构建脚本

set -e

echo "=== Manus Glove Calibration GUI 构建脚本 ==="

# 检查并下载ImGui
if [ ! -d "third_party/imgui" ] || [ ! -f "third_party/imgui/imgui.h" ]; then
    echo "下载ImGui库..."
    ./download_imgui.sh
fi

# 检查系统依赖
echo "检查系统依赖..."
if ! pkg-config --exists glfw3 2>/dev/null; then
    echo "警告: GLFW3未找到，请确保已安装libglfw3-dev"
fi

# 编译
echo "开始编译..."
make clean
make

if [ $? -eq 0 ]; then
    echo ""
    echo "=== 构建成功！ ==="
    echo "运行程序: ./CalibrationGUI.out"
else
    echo ""
    echo "=== 构建失败 ==="
    exit 1
fi

