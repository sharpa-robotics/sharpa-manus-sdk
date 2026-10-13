#!/bin/bash

# 下载ImGui库
IMGUI_VERSION="1.92.5"
IMGUI_DIR="third_party/imgui"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$SCRIPT_DIR"

if [ -d "$IMGUI_DIR" ] && [ -f "$IMGUI_DIR/imgui.h" ]; then
    echo "ImGui already exists"
    exit 0
fi

echo "Downloading ImGui v${IMGUI_VERSION}..."
rm -rf "$IMGUI_DIR"
mkdir -p "$IMGUI_DIR"

# 优先使用git clone
if command -v git &> /dev/null; then
    echo "Using git to clone..."
    git clone --depth 1 --branch "v${IMGUI_VERSION}" https://github.com/ocornut/imgui.git "$IMGUI_DIR"
    
    if [ -f "$IMGUI_DIR/imgui.h" ]; then
        echo "ImGui downloaded successfully"
        exit 0
    fi
    
    echo "Git clone failed, trying wget/curl..."
    rm -rf "$IMGUI_DIR"
    mkdir -p "$IMGUI_DIR"
fi

# 备用方法：使用wget或curl下载
TMP_FILE="/tmp/imgui_v${IMGUI_VERSION}.tar.gz"

if command -v wget &> /dev/null; then
    wget https://github.com/ocornut/imgui/archive/refs/tags/v${IMGUI_VERSION}.tar.gz -O "$TMP_FILE"
elif command -v curl &> /dev/null; then
    curl -L https://github.com/ocornut/imgui/archive/refs/tags/v${IMGUI_VERSION}.tar.gz -o "$TMP_FILE"
else
    echo "Error: git, wget or curl is required"
    exit 1
fi

# 解压
if [ -f "$TMP_FILE" ]; then
    tar -xzf "$TMP_FILE" -C "$IMGUI_DIR" --strip-components=1
    rm -f "$TMP_FILE"
    
    if [ -f "$IMGUI_DIR/imgui.h" ]; then
        echo "ImGui downloaded successfully"
    else
        echo "Error: Failed to extract ImGui"
        exit 1
    fi
else
    echo "Error: Failed to download ImGui"
    exit 1
fi
