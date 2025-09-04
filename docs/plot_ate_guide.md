# ATE 分析脚本使用指南

## 概述

`plot_ate.py` 是用于计算和可视化轨迹 Absolute Trajectory Error (ATE) 的专用脚本，基于 `plot_results.py` 扩展开发。

## 功能特性

- **轨迹对齐**: 支持 SE(2), SE(3), Sim(3) 变换
- **ATE 指标**: 计算 RMSE、mean、median、std、max、min 等统计量
- **可视化**: 生成轨迹对比图、误差时间序列、误差分布直方图
- **数据格式**: 兼容带时间戳和仅位置的轨迹文件
- **输出格式**: PNG图像、JSON摘要、CSV详细数据、NPZ轨迹数据

## 使用方法

### 基本用法
```bash
python3 scripts/plot_ate.py --estimated <估计轨迹文件> --ground_truth <真值轨迹文件>
```

### 完整示例
```bash
# 使用项目样例数据
python3 scripts/plot_ate.py \
  --estimated build/poses_optimized.txt \
  --ground_truth build/poses_original.txt \
  --verbose

# 使用 Sim3 对齐和 3D 视图
python3 scripts/plot_ate.py \
  --estimated traj_est.txt \
  --ground_truth traj_gt.txt \
  --alignment_type sim3 \
  --plot_3d \
  --output_dir my_results

# 保存对齐后的轨迹数据
python3 scripts/plot_ate.py \
  --estimated traj_est.txt \
  --ground_truth traj_gt.txt \
  --save_aligned_traj \
  --prefix my_experiment
```

## 参数说明

### 必需参数
- `--estimated`: 估计轨迹文件路径
- `--ground_truth`: 真值轨迹文件路径

### 对齐参数
- `--alignment_type`: 对齐类型，选项：
  - `se2`: 2D 刚体变换（平移 + 旋转）
  - `se3`: 3D 刚体变换（平移 + 旋转，默认）
  - `sim3`: 3D 相似变换（平移 + 旋转 + 尺度）
- `--max_pairs`: 用于对齐的最大位姿对数，默认 10000

### 输出控制参数
- `--output_dir`: 输出目录路径，默认 `output/ate_analysis`
- `--prefix`: 输出文件前缀，默认 `ate`
- `--dpi`: 图像分辨率，默认 300

### 绘图参数
- `--plot_3d`: 绘制3D轨迹图（默认2D俯视图）
- `--no_show`: 不显示图像窗口

### 数据保存参数
- `--save_aligned_traj`: 保存对齐后的轨迹数据

### 调试参数
- `--verbose`: 详细输出

## 输出文件位置设置

### 1. 默认输出位置

如果不指定 `--output_dir`，所有结果文件将保存到：
```
output/ate_analysis/
```

### 2. 自定义输出目录

使用 `--output_dir` 参数指定输出目录：

```bash
# 相对路径
python3 scripts/plot_ate.py ... --output_dir results/ate_20240904

# 绝对路径
python3 scripts/plot_ate.py ... --output_dir /home/user/experiments/ate_analysis

# 多级目录（会自动创建）
python3 scripts/plot_ate.py ... --output_dir experiments/session_1/ate_results
```

### 3. 输出文件命名规则

所有输出文件都遵循以下命名模式：
```
{prefix}_{type}_{timestamp}.{extension}
```

其中：
- `prefix`: 由 `--prefix` 参数指定，默认 `ate`
- `type`: 文件类型标识
- `timestamp`: 生成时间戳，格式 `YYYYMMDD_HHMMSS`
- `extension`: 文件扩展名

### 4. 具体输出文件列表

#### 图像文件（PNG格式）
```
{output_dir}/{prefix}_trajectories_{2d|3d}_{timestamp}.png     # 轨迹对比图
{output_dir}/{prefix}_errors_timeseries_{timestamp}.png        # 误差时间序列
{output_dir}/{prefix}_errors_histogram_{timestamp}.png         # 误差分布直方图
```

#### 数据文件
```
{output_dir}/{prefix}_metrics_{timestamp}.json                 # ATE指标摘要
{output_dir}/{prefix}_detailed_{timestamp}.csv                 # 详细误差数据
{output_dir}/{prefix}_aligned_trajectories_{timestamp}.npz     # 对齐轨迹（可选）
```

## 输出文件内容详解

### 1. 图像文件含义

#### 轨迹对比图 (`*_trajectories_*.png`)
**用途**: 直观展示轨迹对齐效果和空间分布
**内容**:
- **绿线**: 真值轨迹 (Ground Truth)
- **红虚线**: 原始估计轨迹 (Estimated Original) - 对齐前
- **蓝线**: 对齐后估计轨迹 (Estimated Aligned) - 对齐后

**解读要点**:
- 蓝线与绿线重合度越高，说明对齐效果越好
- 红虚线显示原始偏差，体现对齐的必要性
- 标题显示 ATE RMSE 值，提供定量参考

#### 误差时间序列图 (`*_errors_timeseries_*.png`)
**用途**: 分析误差随时间的变化趋势
**内容**:
- **红色曲线**: 逐时刻的 ATE 误差值
- **蓝色虚线**: RMSE 水平线
- **橙色虚线**: 平均误差水平线
- **绿色虚线**: 中位数误差水平线

**解读要点**:
- 误差曲线平稳说明算法稳定性好
- 误差峰值位置可能对应算法失效时刻
- 误差趋势可识别累积漂移或周期性问题

#### 误差分布直方图 (`*_errors_histogram_*.png`)
**用途**: 分析误差的统计分布特征
**内容**:
- **柱状图**: 误差值的频次分布
- **垂直虚线**: RMSE、平均值、中位数标记

**解读要点**:
- 正态分布说明误差随机性好
- 长尾分布说明存在异常值
- 多峰分布可能表示多种误差模式

### 2. 数据文件内容

#### ATE指标摘要 (`*_metrics_*.json`)
**用途**: 存储完整的数值分析结果
**内容结构**:
```json
{
  "timestamp": "20240904_150101",           // 分析时间戳
  "alignment_type": "se3",                  // 使用的对齐算法
  "num_poses": 484,                         // 参与计算的位姿数量
  "metrics": {                              // 核心ATE指标
    "rmse": 4.696157,                       // 均方根误差（最重要）
    "mean": 3.921774,                       // 平均误差
    "median": 3.342996,                     // 中位数误差
    "std": 2.583328,                        // 标准差
    "max": 13.902171,                       // 最大误差
    "min": 0.020838                         // 最小误差
  },
  "statistics": {                           // 扩展统计信息
    "percentile_25": 2.065382,              // 25%分位数
    "percentile_75": 5.215533,              // 75%分位数
    "percentile_95": 9.114778,              // 95%分位数
    "percentile_99": 12.064175               // 99%分位数
  }
}
```

**字段含义**:
- `rmse`: **最重要的ATE指标**，反映整体轨迹精度
- `mean`: 平均误差，反映系统性偏差
- `median`: 中位数误差，对异常值更稳健
- `std`: 标准差，反映误差的一致性和稳定性
- `max/min`: 识别最大偏差和最佳精度
- `percentile_*`: 识别误差分布的特征点

#### 详细误差数据 (`*_detailed_*.csv`)
**用途**: 存储每个时刻的具体误差值，便于后续分析
**格式**:
```csv
timestamp,ate_error
0.000,1.234567
1.000,2.345678
...
```

**应用场景**:
- 导入其他分析工具
- 自定义统计分析
- 与其他实验结果对比
- 生成自定义图表

#### 对齐轨迹数据 (`*_aligned_trajectories_*.npz`)
**用途**: 保存对齐过程的完整信息，便于复现和深入分析
**内容**:
```python
{
  'timestamps': array([...]),               // 时间戳序列
  'original_estimated': array([...]),       // 原始估计轨迹
  'aligned_estimated': array([...]),        // 对齐后估计轨迹
  'ground_truth': array([...]),             // 真值轨迹
  'rotation_matrix': array([...]),          // 对齐旋转矩阵 (3x3)
  'translation_vector': array([...]),       // 对齐平移向量 (3,)
  'scale_factor': float,                    // 尺度因子
  'alignment_type': 'se3'                   // 对齐算法类型
}
```

**应用场景**:
- 验证对齐算法的正确性
- 应用相同变换到其他数据
- 深入分析轨迹变换参数
- 算法调试和优化

### 3. 文件使用建议

#### 快速评估流程
1. **查看 JSON 摘要** → 获取核心 ATE 指标
2. **查看轨迹对比图** → 直观判断对齐效果
3. **查看误差时间序列** → 识别问题时段
4. **查看误差直方图** → 评估误差分布特征

#### 深入分析流程
1. **加载 NPZ 文件** → 获取完整轨迹和变换参数
2. **分析 CSV 数据** → 自定义统计分析
3. **对比多次实验** → 评估算法稳定性和改进效果

#### 批量处理示例
```python
import numpy as np
import json
import glob

# 批量加载多个实验的ATE结果
results = []
for json_file in glob.glob('output/ate_analysis/*_metrics_*.json'):
    with open(json_file, 'r') as f:
        data = json.load(f)
        results.append({
            'experiment': json_file,
            'rmse': data['metrics']['rmse'],
            'mean': data['metrics']['mean'],
            'num_poses': data['num_poses']
        })

# 对比分析
for result in results:
    print(f"{result['experiment']}: RMSE={result['rmse']:.4f}m")
```

### 4. 输出文件实际示例

#### 典型输出目录结构
```
output/ate_analysis/
├── ate_trajectories_2d_20240904_143022.png    (806 KB) - 轨迹对比图
├── ate_errors_timeseries_20240904_143022.png   (322 KB) - 误差时序图  
├── ate_errors_histogram_20240904_143022.png    (103 KB) - 误差直方图
├── ate_metrics_20240904_143023.json            (477 B)  - 核心指标
├── ate_detailed_20240904_143023.csv            (12 KB)  - 详细数据
└── ate_aligned_trajectories_20240904_143023.npz (40 KB) - 轨迹数据(可选)
```

#### JSON 文件示例内容
```json
{
  "timestamp": "20240904_150101",
  "alignment_type": "se3", 
  "num_poses": 484,
  "metrics": {
    "rmse": 4.696157,        // 主要评估指标
    "mean": 3.921774,        // 平均误差 
    "median": 3.342996,      // 中位数误差
    "std": 2.583328,         // 标准差
    "max": 13.902171,        // 最大误差点
    "min": 0.020838          // 最小误差点
  },
  "statistics": {
    "percentile_25": 2.065,  // 25%的误差小于此值
    "percentile_75": 5.216,  // 75%的误差小于此值  
    "percentile_95": 9.115,  // 95%的误差小于此值
    "percentile_99": 12.064  // 99%的误差小于此值
  }
}
```

#### CSV 文件示例内容
```csv
timestamp,ate_error
0.000,1.234567          // 第0时刻的ATE误差
1.000,2.345678          // 第1时刻的ATE误差
2.000,1.876543          // 第2时刻的ATE误差
...
483.000,3.456789        // 最后时刻的ATE误差
```

#### NPZ 文件加载示例
```python
import numpy as np

# 加载对齐轨迹数据
data = np.load('ate_aligned_trajectories_20240904_143023.npz')

print("可用数据字段:", list(data.keys()))
print("旋转矩阵:", data['rotation_matrix'])
print("平移向量:", data['translation_vector']) 
print("尺度因子:", data['scale_factor'])

# 验证变换关系
original_est = data['original_estimated']
aligned_est = data['aligned_estimated']
R, t, s = data['rotation_matrix'], data['translation_vector'], data['scale_factor']

# 手动应用变换
manual_aligned = s * (original_est @ R.T) + t
print("变换验证误差:", np.max(np.abs(manual_aligned - aligned_est)))
```

### 5. 输出目录示例

#### 示例1：默认设置
```bash
python3 scripts/plot_ate.py --estimated traj_est.txt --ground_truth traj_gt.txt
```
输出位置：
```
output/ate_analysis/
├── ate_trajectories_2d_20240904_143022.png
├── ate_errors_timeseries_20240904_143022.png
├── ate_errors_histogram_20240904_143022.png
├── ate_metrics_20240904_143023.json
└── ate_detailed_20240904_143023.csv
```

#### 示例2：自定义目录和前缀
```bash
python3 scripts/plot_ate.py \
  --estimated traj_est.txt \
  --ground_truth traj_gt.txt \
  --output_dir experiments/session_1 \
  --prefix slam_ate \
  --save_aligned_traj
```
输出位置：
```
experiments/session_1/
├── slam_ate_trajectories_2d_20240904_143022.png
├── slam_ate_errors_timeseries_20240904_143022.png
├── slam_ate_errors_histogram_20240904_143022.png
├── slam_ate_metrics_20240904_143023.json
├── slam_ate_detailed_20240904_143023.csv
└── slam_ate_aligned_trajectories_20240904_143023.npz
```

#### 示例3：按日期组织
```bash
# 为不同实验创建按日期分组的目录
python3 scripts/plot_ate.py \
  --estimated exp1_traj.txt \
  --ground_truth gt.txt \
  --output_dir results/2024-09-04/experiment_1 \
  --prefix exp1_ate

python3 scripts/plot_ate.py \
  --estimated exp2_traj.txt \
  --ground_truth gt.txt \
  --output_dir results/2024-09-04/experiment_2 \
  --prefix exp2_ate
```

### 6. 目录权限与创建

- 脚本会自动创建不存在的输出目录
- 确保对目标目录有写入权限
- 支持多级目录创建（如 `results/session_1/ate_analysis`）

## 支持的数据格式

### 带时间戳格式
```
timestamp x y z [qx qy qz qw]
0.000 14.604 31.768 0.000 0.000 0.000 0.639 0.770
1.000 15.526 70.679 0.000 0.000 0.000 0.707 0.707
...
```

### 仅位置格式
```
x y z
14.604 31.768 0.000
15.526 70.679 0.000
...
```

## ATE 计算原理

### 1. 轨迹关联
- 基于时间戳最近邻匹配
- 默认最大时间差阈值：0.02秒
- 支持无时间戳的序号对齐

### 2. 轨迹对齐
使用 Umeyama 算法求解最优变换：

**SE(3) 刚体变换**:
```
Y = R * X + t
```
其中 R 是旋转矩阵，t 是平移向量

**Sim(3) 相似变换**:
```
Y = s * R * X + t
```
其中额外包含尺度因子 s

### 3. ATE 计算
对齐后计算位置误差：
```
ATE(i) = ||y_i - (s * R * x_i + t)||
```

### SE(2), SE(3), Sim(3) 的含义与区别

1. SE(2)（Special Euclidean group in 2D）

作用：平面刚体变换（二维旋转 + 平移）
形式：T = [ R(θ) t; 0 1 ]，R ∈ SO(2)，t ∈ R²
自由度：3（旋转1 + 平移2）
常用于：移动机器人、平面SLAM、2D路径规划

2. SE(3)（Special Euclidean group in 3D）

作用：三维刚体变换（3D旋转 + 平移）
形式：T = [ R t; 0 1 ]，R ∈ SO(3)，t ∈ R³
自由度：6（旋转3 + 平移3）
常用于：视觉/激光SLAM、机械臂、AUV/无人机位姿表示

3. Sim(3)（Similarity group in 3D）

作用：带统一尺度的相似变换（缩放 + 旋转 + 平移）
形式：T = [ s·R t; 0 1 ]，s > 0，R ∈ SO(3)，t ∈ R³
自由度：7（尺度1 + 旋转3 + 平移3）
常用于：单目SLAM（因尺度漂移）、多源地图对齐、全局尺度校准

**核心区别：**

SE(2)/SE(3)：保持形状与长度（刚体），不改变尺度
Sim(3)：允许统一尺度伸缩，适合存在尺度不确定或漂移的情况
若你的传感器具有真实尺度（如多波束、激光、双目/带IMU融合），通常用 SE(3)；若可能存在尺度模糊（如纯单目），需用 Sim(3)

## 典型结果解读

### ATE 指标含义
```json
{
  "rmse": 4.696157,      // 均方根误差，最重要的ATE指标
  "mean": 3.921774,      // 平均误差
  "median": 3.342996,    // 中位数误差，更稳健的中心趋势
  "std": 2.583328,       // 标准差，反映误差一致性
  "max": 13.902171,      // 最大误差，识别异常点
  "min": 0.020838,       // 最小误差
  "percentile_95": 9.115 // 95%分位数，识别大误差阈值
}
```

### 结果评估准则
- **RMSE < 1m**: 优秀的轨迹精度
- **RMSE 1-5m**: 良好的轨迹精度
- **RMSE > 10m**: 需要检查算法或数据质量
- **std/mean < 0.5**: 误差分布相对一致
- **max/median > 5**: 存在显著异常点

## 常见问题与解决方案

### 1. 文件格式错误
```
错误: 不支持的数据格式，列数: 2，期望至少3列
```
**解决**: 检查轨迹文件是否包含至少 x, y, z 三列位置数据

### 2. 时间戳不匹配
```
错误: 无法关联轨迹，时间差阈值过小: 0.02s
```
**解决**: 
- 检查两个轨迹文件的时间戳格式是否一致
- 或使用无时间戳格式（仅位置坐标）

### 3. 轨迹长度差异
```
Warning: Length mismatch between estimated (500) and ground_truth (484)
```
**解决**: 脚本会自动处理长度不一致，使用较短轨迹的长度

### 4. Sim3 对齐结果异常

#### 问题现象
使用 `--alignment_type sim3` 时可能出现 RMSE 异常大的情况：
```
SE3 对齐: ATE RMSE: 4.696157 m      ← 正常
Sim3 对齐: ATE RMSE: 169831.953 m   ← 异常大
```

#### 根本原因
**Umeyama 算法在 Sim3 模式下的尺度计算缺陷**：

1. **数值稳定性问题**: 
   - 轨迹跨度大（~870m）导致协方差矩阵奇异值过大（3200万级别）
   - 尺度因子 = 奇异值迹 / 轨迹方差 = 59499354 / 122233 = 486.77（异常大）

2. **算法适用性问题**:
   - 当前数据的估计轨迹与真值轨迹尺度本身匹配良好（方差比 0.988）
   - 强制使用 Sim3 反而引入不必要的尺度变换

#### 诊断方法
检查以下输出参数：
```bash
python3 scripts/plot_ate.py ... --alignment_type sim3 --verbose
```

关注这些数值：
- **尺度因子**: 应接近 1.0，如果偏离过大（>1.5 或 <0.5）说明不适用
- **平移向量**: Sim3 的平移向量异常大说明尺度计算错误
- **质心距离**: 两轨迹质心距离应该较小

#### 解决方案
- **推荐**: 对于 SLAM 生成的轨迹，优先使用 **SE3 刚体变换**
- **Sim3 适用场景**: 
  - 单目 SLAM（尺度不确定）
  - 不同传感器融合（存在真实尺度差异）
  - 地图拼接（需要尺度校正）
- **判断准则**: 如果两轨迹的空间跨度相近，使用 SE3；如果存在明显尺度差异，才使用 Sim3

## 高级用法

### 批量处理多个轨迹
```bash
#!/bin/bash
# 批量处理脚本示例

for exp in exp1 exp2 exp3; do
  python3 scripts/plot_ate.py \
    --estimated results/${exp}_trajectory.txt \
    --ground_truth ground_truth.txt \
    --output_dir results/ate_analysis/${exp} \
    --prefix ${exp}_ate \
    --verbose \
    --no_show
done
```

### 性能优化
对于超长轨迹（>10000个位姿），建议：
```bash
python3 scripts/plot_ate.py \
  --estimated long_traj.txt \
  --ground_truth gt.txt \
  --max_pairs 5000 \
  --no_show
```

### 集成到评估流水线
```bash
# 1. 运行SLAM算法
./bin/bathy_slam_real config.yaml

# 2. 分析ATE
python3 scripts/plot_ate.py \
  --estimated build/poses_optimized.txt \
  --ground_truth build/poses_original.txt \
  --output_dir evaluation/$(date +%Y%m%d_%H%M%S) \
  --save_aligned_traj \
  --verbose
```

## 与其他脚本的关系

- **plot_results.py**: 用于轨迹可视化对比
- **plot_ate.py**: 专门用于 ATE 误差分析
- **config.yaml**: 包含SLAM算法参数，ATE分析独立于此配置

建议工作流：
1. 使用 `plot_results.py` 进行轨迹可视化检查
2. 使用 `plot_ate.py` 进行定量ATE分析
3. 结合两者结果评估SLAM算法性能