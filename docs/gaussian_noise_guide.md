# 高斯噪声实现与随机种子指南

本指南详细介绍了Bathymetric SLAM中高斯噪声的实现机制、配置方法以及可复现性功能的使用。

## 概述

高斯噪声在SLAM系统中用于模拟真实传感器数据中的不确定性和误差，帮助测试算法在存在噪声环境下的鲁棒性。本项目在航迹推算（Dead Reckoning, DR）边上添加高斯噪声，以评估SLAM算法对位姿估计误差的容忍能力。

## 相关配置参数

在 `config.yaml` 文件中，有两个与高斯噪声相关的参数：

```yaml
add_gaussian_noise: true    # 是否在模拟数据中添加高斯噪声
noise_seed: 42             # 可选的随机噪声种子，注释掉或留空则为完全随机
```

### add_gaussian_noise
- **类型**: `boolean`
- **默认值**: `true`
- **功能**: 控制是否在SLAM图的DR边上添加高斯噪声
- **影响**: 当设置为 `true` 时，系统会在每条DR边的航向角（yaw）上添加小幅随机扰动

### noise_seed
- **类型**: `integer`（可选）
- **默认值**: `42`（示例值）
- **功能**: 设置随机数生成器的种子，实现噪声的可复现性
- **使用方法**:
  - 设置具体数值：每次运行产生相同的噪声模式
  - 注释掉或删除：保持完全随机的噪声生成

## 实现原理

### 1. 噪声类型与分布

当前实现的高斯噪声具有以下特点：

- **噪声配置**: 系统配置了完整的6自由度噪声参数（3个平移 + 3个旋转）
- **实际应用**: 只在旋转的yaw轴（航向角）上实际添加噪声
- **噪声分布**: 正态分布，均值为0，标准差为0.01弧度
- **影响范围**: 所有DR（Dead Reckoning）边
- **被禁用的噪声**: 平移噪声和roll/pitch旋转噪声在代码中被注释掉，设置为零

### 2. 噪声参数与随机种子的作用机制

理解噪声参数和随机种子之间的关系对于正确使用高斯噪声功能至关重要：

#### 2.1 噪声参数的作用

噪声参数定义的是**噪声的统计特性**，即噪声分布的"规格"：

```cpp
// 在 generateGaussianNoise() 函数中
std::vector<double> noiseTranslation = {3, 3, 0.001};      // 平移噪声的标准差
std::vector<double> noiseRotation = {0.0001, 0.0001, 0.001}; // 旋转噪声的标准差

// 构建协方差矩阵（方差 = 标准差²）
Eigen::Matrix3d transNoise = Eigen::Matrix3d::Zero();
for (int i = 0; i < 3; ++i)
  transNoise(i, i) = std::pow(noiseTranslation[i], 2);
```

这些参数决定了：
- **噪声强度**：数值越大，噪声幅度越大
- **分布形状**：定义多维高斯分布的协方差矩阵  
- **相对比例**：不同轴向噪声强度的相对关系

#### 2.2 随机种子的作用

随机种子决定的是**从噪声分布中具体采样什么数值**：

```cpp
// 设置噪声采样器的分布参数（定义"规格"）
transSampler.setDistribution(transNoise);  
rotSampler.setDistribution(rotNoise);

// 随机种子决定从分布中采样的具体数值序列
if (isNoiseSeedSet()) {
    std::mt19937& rng = getGlobalNoiseRNG();
    transSampler.seed(rng());  // 使用确定性种子
    rotSampler.seed(rng());
} else {
    // 使用硬件随机数源产生的随机种子
    std::random_device r;
    std::seed_seq seedSeq{r(), r(), r(), r(), r()};
    vector<int> seeds(2);
    seedSeq.generate(seeds.begin(), seeds.end());
    transSampler.seed(seeds[0]);
    rotSampler.seed(seeds[1]);
}
```

#### 2.3 为什么没有种子时每次效果不同？

在没有设置 `noise_seed` 之前，每次运行的随机化过程：

1. **硬件随机源**：`std::random_device` 产生真正的随机数（来自硬件熵源）
2. **种子生成**：每次运行产生不同的种子序列
3. **结果差异**：相同噪声参数 + 不同种子 = 不同的噪声数值序列

**具体例子**：

假设yaw噪声参数固定为标准差0.01：

```
运行1（随机种子=12345）：yaw噪声序列 [0.008, -0.003, 0.011, -0.007, ...]
运行2（随机种子=67890）：yaw噪声序列 [-0.004, 0.009, -0.002, 0.012, ...]
运行3（随机种子=24681）：yaw噪声序列 [0.006, -0.008, 0.004, -0.001, ...]
```

虽然都符合相同的统计分布（均值0，标准差0.01），但具体数值完全不同。

#### 2.4 设置种子后的确定性机制

当设置了 `noise_seed: 42` 后：

```cpp
// 确定性初始化过程
setNoiseRandomSeed(42);  // 全局RNG始终用种子42初始化

// 每次运行时
std::mt19937& rng = getGlobalNoiseRNG();  // 总是相同的初始状态
transSampler.seed(rng());                 // 总是得到相同的种子值A
rotSampler.seed(rng());                   // 总是得到相同的种子值B
```

**结果**：
- 每次运行，采样器的种子完全相同
- 相同参数 + 相同种子 = **完全相同的噪声序列**
- 实现了噪声的可复现性

#### 2.5 实际应用示例

在图边噪声添加中的体现：

```cpp
void GraphConstructor::addNoiseToGraph(GaussianGen& transSampler, GaussianGen& rotSampler){
    std::mt19937& gen = getGlobalNoiseRNG();  // 获取（可能确定性的）随机数生成器
    std::normal_distribution<> d{0, 0.01};    // yaw噪声分布：均值0，标准差0.01
    
    for (size_t i = 0; i < drEdges_.size(); ++i) {
        double yaw = d(gen);  // 从分布中采样yaw噪声值
        // ... 将噪声应用到第i条边的测量值
    }
}
```

- **无种子**：每次运行，`gen`状态不同，`d(gen)`产生不同序列
- **有种子**：每次运行，`gen`初始状态相同，`d(gen)`产生相同序列

#### 2.6 类比理解

可以这样理解噪声参数与随机种子的关系：

- **噪声参数**：定义了"从什么样的袋子里摸球"（袋子里球的分布）
- **随机种子**：决定了"按什么顺序摸球"（摸球的具体序列）
- **可复现性**：固定种子 = 每次都按相同顺序摸球 = 得到相同结果
- **随机性**：不固定种子 = 每次摸球顺序随机 = 符合统计规律但结果不同

### 3. 代码实现架构

高斯噪声的实现分布在以下几个关键文件中：

#### 3.1 全局随机数管理 (`utils_g2o.hpp/cpp`)
```cpp
// 核心API
void setNoiseRandomSeed(int seed);      // 设置全局随机种子
bool isNoiseSeedSet();                  // 检查种子是否已设置
std::mt19937& getGlobalNoiseRNG();      // 获取全局随机数生成器
```

#### 3.2 噪声生成器初始化 (`generateGaussianNoise`)
- 定义平移和旋转的噪声协方差矩阵
- 根据是否设置种子来决定采样器的初始化方式
- 返回信息矩阵供后续使用

#### 3.3 图边噪声添加 (`GraphConstructor::addNoiseToGraph`)
- 遍历所有DR边
- 对每条边的航向角添加正态分布噪声
- 更新边的测量值

### 4. 执行流程

1. **配置加载**: 程序启动时从 `config.yaml` 读取噪声相关配置
2. **种子设置**: 如果配置了 `noise_seed`，则调用 `setNoiseRandomSeed()` 初始化全局随机数生成器
3. **采样器初始化**: `generateGaussianNoise()` 函数根据种子设置状态初始化高斯采样器
4. **噪声添加**: 如果 `add_gaussian_noise` 为 `true`，在图构建过程中调用 `addNoiseToGraph()` 添加噪声

## 使用指南

### 启用高斯噪声

在 `config.yaml` 中设置：
```yaml
add_gaussian_noise: true
```

### 实现可复现的噪声

为了获得可重复的实验结果，设置固定的随机种子：
```yaml
add_gaussian_noise: true
noise_seed: 12345          # 使用任意整数作为种子
```

每次运行程序时，将产生完全相同的噪声模式。

### 使用完全随机的噪声

如果需要每次运行都产生不同的噪声：
```yaml
add_gaussian_noise: true
# noise_seed: 12345        # 注释掉或删除这一行
```

或者完全删除 `noise_seed` 配置项。

### 禁用高斯噪声

如果不需要添加噪声：
```yaml
add_gaussian_noise: false
# noise_seed的设置在此情况下不起作用
```

## 技术细节

### 噪声参数（硬编码）

当前版本中，噪声的具体参数在代码中硬编码：

```cpp
// 在 generateGaussianNoise() 函数中 - 配置完整的噪声参数
std::vector<double> noiseTranslation = {3, 3, 0.001};      // x, y, z平移噪声（已配置但未使用）
std::vector<double> noiseRotation = {0.0001, 0.0001, 0.001}; // roll, pitch, yaw旋转噪声（已配置但仅yaw被使用）

// 在 addNoiseToGraph() 函数中 - 实际使用的噪声
std::normal_distribution<> d{0, 0.01};  // 实际应用的yaw噪声分布
// trans = transSampler.generateSample();  // 平移噪声被注释掉
// trans.setZero();                        // 平移设为零
```

**重要说明**: 虽然系统配置了完整的6DOF噪声采样器，但实际代码中只应用了yaw轴的旋转噪声。平移噪声和roll/pitch旋转噪声都被显式禁用。

### 随机数生成器选择

- **类型**: `std::mt19937`（梅森旋转算法）
- **优点**: 高质量伪随机数，周期长，统计特性好
- **种子管理**: 通过全局单例模式确保整个程序使用统一的随机数源

### 噪声应用时机

噪声在以下阶段被应用：
1. **图构建完成后**: 在所有DR边创建完毕后统一添加噪声
2. **优化开始前**: 确保图优化算法处理的是已加噪的测量值
3. **一次性操作**: 噪声只在初始图构建时添加一次

## 扩展与定制

### 修改噪声参数

如需调整噪声的强度或类型，可以修改以下代码：

1. **调整yaw噪声强度**（`graph_construction.cpp`）:
```cpp
std::normal_distribution<> d{0, 0.02};  // 将标准差从0.01改为0.02
```

2. **启用平移噪声**（`graph_construction.cpp` 和 `utils_g2o.cpp`）:
```cpp
// 在 addNoiseToGraph() 函数中
Eigen::Vector3d trans = transSampler.generateSample();  // 取消注释
// trans.setZero();  // 注释掉这一行

// 同样在 addNoiseToSubmap() 和 addNoiseToMap() 函数中做相同修改
```

3. **启用roll/pitch旋转噪声**（`graph_construction.cpp`）:
```cpp
// 使用 rotSampler 生成的完整旋转噪声
Eigen::Vector3d quatXYZ = rotSampler.generateSample();  // 已存在
double qw = 1.0 - quatXYZ.norm();
if (qw < 0) qw = 0.;
Eigen::Quaterniond rot(qw, quatXYZ.x(), quatXYZ.y(), quatXYZ.z());  // 取消注释
// 注释掉仅yaw噪声的部分
```

4. **修改噪声参数**（`utils_g2o.cpp`）:
```cpp
// 调整 generateGaussianNoise() 函数中的噪声强度
noiseTranslation = {1.0, 1.0, 0.05};  // 减小平移噪声
noiseRotation = {0.01, 0.01, 0.02};   // 增大旋转噪声
```

### 从配置文件读取噪声参数

未来版本可以考虑将硬编码的噪声参数移至 `config.yaml`：
```yaml
noise_parameters:
  translation_std: [3.0, 3.0, 0.001]
  rotation_std: [0.0001, 0.0001, 0.001]
  yaw_bias_std: 0.01
```

## 故障排除

### 编译错误
- 确保所有相关头文件正确包含
- 检查 `std::mt19937` 拼写是否正确

### 运行时问题
- 验证 `config.yaml` 语法正确
- 确认 `noise_seed` 为整数类型
- 检查程序输出中的种子设置确认信息

### 结果验证
- 使用相同种子多次运行，确认结果一致性
- 比较有噪声和无噪声运行的轨迹差异
- 通过基准测试评估噪声对SLAM性能的影响

## 参考信息

- 相关源码文件：
  - `src/graph_optimization/src/utils_g2o.cpp`
  - `src/graph_optimization/src/graph_construction.cpp` 
  - `src/apps/src/test_slam_real.cpp`
- 配置文件：`config.yaml`
- 相关类型定义：`GaussianGen` (g2o::GaussianSampler)
