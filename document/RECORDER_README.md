# 图像录制节点使用说明

## 概述

`mvsdk_ros2_recorder_node` 是一个用于录制 `mv/raw_image` 话题图像的 ROS 2 节点。它可以将接收到的 `sensor_msgs::msg::Image` 消息保存为图像文件。

## 功能特性

- 订阅 `mv/raw_image` 话题
- 支持多种图像格式（jpg, png, bmp 等）
- 可配置输出目录
- 支持运行时控制录制开始/停止
- 自动生成带时间戳的文件名
- 可配置自动开始录制

## 启动方式

### 1. 单独启动录制节点

```bash
ros2 run mvsdk_ros2 mvsdk_ros2_recorder_node
```

### 2. 使用启动文件（同时启动相机和录制节点）

```bash
ros2 launch mvsdk_ros2 camera_recorder.py
```

### 3. 带参数启动

```bash
ros2 launch mvsdk_ros2 camera_recorder.py \
    output_dir:=my_recordings \
    image_format:=png \
    auto_start_recording:=true
```

## 参数配置

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `output_dir` | string | "recorded_images" | 图像保存目录 |
| `image_format` | string | "jpg" | 图像格式（jpg, png, bmp等） |
| `auto_start_recording` | bool | false | 是否自动开始录制 |

## 服务接口

### 开始录制
```bash
ros2 service call /start_recording std_srvs/srv/Empty
```

### 停止录制
```bash
ros2 service call /stop_recording std_srvs/srv/Empty
```

## 输出文件命名规则

文件名格式：`image_YYYYMMDD_HHMMSS_MMM_NNNNNN.格式`

- `YYYYMMDD_HHMMSS`：年月日_时分秒
- `MMM`：毫秒（3位）
- `NNNNNN`：图像序号（6位，从0开始）

示例：`image_20250811_154530_123_000001.jpg`

## 使用示例

### 示例1：基本录制

1. 启动相机节点：
```bash
ros2 run mvsdk_ros2 mvsdk_ros2_node
```

2. 启动录制节点：
```bash
ros2 run mvsdk_ros2 mv_recorder_node
```

3. 开始录制：
```bash
ros2 service call /start_recording std_srvs/srv/Empty
```

4. 停止录制：
```bash
ros2 service call /stop_recording std_srvs/srv/Empty
```
### 检查话题是否有数据

```bash
ros2 topic echo /mv/raw_image
```

### 检查节点状态

```bash
ros2 node info /mv_recorder
```
