# 输出文件格式说明 / Output File Format Documentation

## 概述 / Overview

该项目在运行过程中会生成多个 `.txt` 文件，用于存储不同阶段的位姿信息。这些文件采用统一的格式，便于后续的可视化和分析。

This project generates multiple `.txt` files during execution to store pose information at different stages. These files use a unified format for subsequent visualization and analysis.

## 生成的文件 / Generated Files

项目会生成以下三个主要的位姿文件：

The project generates the following three main pose files:

1. **`poses_original.txt`** - 原始位姿数据 / Original pose data
2. **`poses_corrupted.txt`** - 带噪声的位姿数据 / Corrupted pose data with noise  
3. **`poses_optimized.txt`** - 优化后的位姿数据 / Optimized pose data after Ceres optimization

## 文件格式 / File Format

每个文件都采用相同的格式，每行代表一个位姿：

Each file uses the same format, with each line representing one pose:

```
id x y z q_x q_y q_z q_w
```

### 字段说明 / Field Description

| 字段 / Field | 类型 / Type | 说明 / Description |
|--------------|-------------|-------------------|
| `id` | 整数 / Integer | 位姿ID，通常从0开始递增 / Pose ID, typically starting from 0 |
| `x` | 浮点数 / Float | X坐标位置 (米) / X position coordinate (meters) |
| `y` | 浮点数 / Float | Y坐标位置 (米) / Y position coordinate (meters) |
| `z` | 浮点数 / Float | Z坐标位置 (米) / Z position coordinate (meters) |
| `q_x` | 浮点数 / Float | 四元数X分量 / Quaternion X component |
| `q_y` | 浮点数 / Float | 四元数Y分量 / Quaternion Y component |
| `q_z` | 浮点数 / Float | 四元数Z分量 / Quaternion Z component |
| `q_w` | 浮点数 / Float | 四元数W分量 / Quaternion W component |

### 示例 / Example

```
0 10.500000 20.300000 -5.200000 0.000000 0.000000 0.000000 1.000000
1 11.200000 21.100000 -5.100000 0.100000 0.000000 0.000000 0.995000
2 12.000000 22.500000 -4.900000 0.200000 0.000000 0.000000 0.980000
```

在这个示例中：
- 位姿0: 位于 (10.5, 20.3, -5.2)，无旋转 (四元数为 [0,0,0,1])
- 位姿1: 位于 (11.2, 21.1, -5.1)，有轻微旋转
- 位姿2: 位于 (12.0, 22.5, -4.9)，旋转角度更大

In this example:
- Pose 0: Located at (10.5, 20.3, -5.2) with no rotation (quaternion [0,0,0,1])
- Pose 1: Located at (11.2, 21.1, -5.1) with slight rotation  
- Pose 2: Located at (12.0, 22.5, -4.9) with more rotation

## 坐标系统 / Coordinate System

- **位置坐标** / **Position coordinates**: 采用右手坐标系，单位为米 / Right-hand coordinate system, units in meters
- **方向表示** / **Orientation representation**: 使用四元数 (x, y, z, w) 表示旋转 / Uses quaternion (x, y, z, w) to represent rotation

## 数据处理说明 / Data Processing Notes

1. **内部存储** / **Internal storage**: 位姿在内部以欧拉角形式存储，输出时转换为四元数 / Poses are stored internally as Euler angles and converted to quaternions for output

2. **数据来源** / **Data source**: 
   - `poses_original.txt`: 来自SLAM系统的原始轨迹估计 / Original trajectory estimates from SLAM system
   - `poses_corrupted.txt`: 优化前的有噪声位姿 / Noisy poses before optimization  
   - `poses_optimized.txt`: 经过Ceres求解器优化后的位姿 / Poses optimized by Ceres solver

3. **可视化** / **Visualization**: 这些文件可以使用 `scripts/plot_results.py` 脚本进行可视化 / These files can be visualized using the `scripts/plot_results.py` script

## 使用方法 / Usage

### 可视化轨迹 / Visualizing Trajectories

```bash
python scripts/plot_results.py \
    --initial_poses poses_original.txt \
    --corrupted_poses poses_corrupted.txt \
    --optimized_poses poses_optimized.txt
```

### 读取数据 / Reading Data

在Python中读取位姿数据：

Reading pose data in Python:

```python
import numpy as np

# 读取所有数据 / Read all data
data = np.loadtxt('poses_original.txt')
ids = data[:, 0]          # 位姿ID / Pose IDs
positions = data[:, 1:4]  # 位置 (x, y, z) / Positions (x, y, z)  
quaternions = data[:, 4:8] # 四元数 (qx, qy, qz, qw) / Quaternions (qx, qy, qz, qw)

# 或仅读取位置信息 / Or read only position information
positions = np.genfromtxt('poses_original.txt', usecols=(1, 2, 3))
```

## 实现细节 / Implementation Details

文件输出功能位于 `src/graph_optimization/src/ceres_optimizer.cpp` 中的 `OutputPoses` 函数。

The file output functionality is implemented in the `OutputPoses` function in `src/graph_optimization/src/ceres_optimizer.cpp`.

```cpp
// 输出格式: id x y z q_x q_y q_z q_w
// Output format: id x y z q_x q_y q_z q_w
bool OutputPoses(const std::string& filename, const MapOfPoses& poses);
```