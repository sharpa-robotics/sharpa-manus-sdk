# Manus Glove Calibration GUI

基于Manus SDK的可视化标定应用程序。

## 功能特性

- 实时显示左右手套的连接状态和关节角度
- 图形化标定流程界面
- 支持键盘快捷键（F5开始标定，F9下一步）
- 自动步骤管理和提示

## 依赖要求

### 系统依赖

在Ubuntu/Debian系统上安装：

```bash
sudo apt-get update
sudo apt-get install build-essential libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
```

在CentOS/RHEL系统上安装：

```bash
sudo yum install gcc-c++ glfw-devel mesa-libGL-devel mesa-libGLU-devel
```

### 项目依赖

- ManusSDK库（已包含在ManusSDK目录中）
- ImGui库（会自动下载）

## 编译

### 方法1：使用构建脚本（推荐）

```bash
./build.sh
```

构建脚本会自动：
- 检查并下载ImGui库
- 检查系统依赖
- 编译项目

### 方法2：手动编译

1. 下载ImGui库（如果尚未下载）：
```bash
./download_imgui.sh
```

2. 编译项目：
```bash
make
```

3. 运行：
```bash
./CalibrationGUI.out
```

## 使用方法

1. **选择手套**：点击"Left Glove"或"Right Glove"按钮选择要标定的手套
2. **开始标定**：点击"Start Calibration"按钮或按F5键开始标定
3. **执行步骤**：按照提示做出手势，然后点击"Next Step"按钮或按F9键进入下一步
4. **完成标定**：最后一步会自动完成标定流程

## 界面说明

- **顶部状态栏**：显示左右手套的连接状态和实时关节角度
- **左侧信息区**：显示当前标定步骤的详细信息
- **右侧按钮面板**：包含所有操作按钮

## 注意事项

- 确保Manus Core正在运行
- 确保手套已正确连接并配对
- 标定过程中请按照提示做出准确的手势

