# `scripts/plot_results.py` 脚本使用指南

## 1. 脚本功能

`plot_results.py` 是一个用于可视化和评估 SLAM (Simultaneous Localization and Mapping) 算法性能的 Python 脚本。它主要用于比较三组不同的机器人位姿（姿态）数据：

*   **原始位姿 (Ground Truth)**: 通常指无误差的、真实的机器人运动轨迹。
*   **损坏位姿 (Corrupted Poses)**: 模拟的或在 SLAM 优化前带有噪声的轨迹。
*   **优化位姿 (Optimized Poses)**: 经过 SLAM 算法优化校正后的轨迹。

该脚本的核心功能包括：

1.  **读取位姿文件**：从文本文件中加载上述三种位姿数据。
2.  **计算误差**：计算“损坏位姿”和“优化位姿”分别与“原始位姿”之间的平均欧氏距离误差，用于量化评估算法的精度。
3.  **三维可视化**：使用 `matplotlib` 库在三维空间中绘制这三条轨迹，并用不同颜色加以区分，以便直观比较：
    *   **绿色**: 原始位姿 (Ground Truth)
    *   **红色**: 损坏位姿 (Corrupted)
    *   **蓝色**: 优化位姿 (Optimized)

## 2. 环境依赖

在运行此脚本之前，请确保您的 Python 环境中已安装以下库：

*   `numpy`: 用于高效的数组和矩阵运算。
*   `matplotlib`: 用于生成高质量的图表和可视化。

您可以使用 `pip` 包管理器轻松安装它们：

```bash
pip install numpy matplotlib
```

## 3. 如何运行

通过命令行终端执行此脚本，并使用特定参数指定输入的位姿数据文件。

### 命令行参数

| 参数 | 缩写 | 描述 |
| :--- | :--- | :--- |
| `--initial_poses <file>` | | 指定包含原始位姿（Ground Truth）的文件路径。 |
| `--corrupted_poses <file>` | | 指定包含损坏或未优化位姿的文件路径。 |
| `--optimized_poses <file>` | | 指定包含优化后位姿的文件路径。 |
| `--axes_equal` | `-e` | (当前版本未在3D图中完全生效) 尝试使坐标轴的比例相等。 |
| `--output_file <file>` | | (代码中已注释) 将计算出的误差结果保存到指定文件。 |

### 位姿文件格式

脚本期望输入的位姿文件是纯文本格式，每行代表一个位姿。脚本会解析每行的第 **2、3、4** 列作为位姿的 **(x, y, z)** 坐标。文件格式通常如下（例如 g2o 格式的顶点）：

```
VERTEX_SE3:QUAT 0 0.0 0.0 0.0 0.0 0.0 0.0 1.0
VERTEX_SE3:QUAT 1 1.05 -0.02 0.01 0.0 0.0 0.0 1.0
...
```

## 4. 使用示例

假设您的位姿文件位于 `build/` 目录下，文件名分别为：

*   `poses_original.txt`
*   `poses_corrupted.txt`
*   `poses_optimized.txt`

您可以从项目根目录运行以下命令来生成比较图：

```bash
python3 ./scripts/plot_results.py \
    --initial_poses ./build/poses_original.txt \
    --corrupted_poses ./build/poses_corrupted.txt \
    --optimized_poses ./build/poses_optimized.txt
```

## 5. 输出结果

执行脚本后，您将获得两种形式的输出：

### 1. 终端输出

终端会打印出两条误差信息，显示损坏路径和优化路径分别与真实路径的平均偏差。这个数值越小，说明路径精度越高。

```
Diff original and corrupted 5.324
Diff original and optimized 0.128
```

这个结果清晰地表明，优化算法将路径误差从 5.324 大幅降低到了 0.128。

### 2. 可视化图窗

一个 `matplotlib` 绘图窗口将会弹出，其中包含一个3D图表，标题为 "Trajectories: GT (Green), Corrupted (Red), optimized (Blue)"。

*   **绿色线条** 是基准的真实路径。
*   **红色线条** 是带有噪声的路径，通常会明显偏离绿色线条。
*   **蓝色线条** 是经过 SLAM 优化的路径，理想情况下它应该与绿色线条高度重合。

通过此图，您可以直观地评估 SLAM 算法的修正效果。

## 6. 综合调用示例

以下是一个完整的示例，展示了如何从运行 SLAM 模拟到最终可视化结果的全过程。

### 第 1 步：运行 SLAM 程序生成位姿文件

首先，我们需要运行主程序 `bathy_slam_real` 来处理模拟数据并生成位姿文件。根据 `.vscode/launch.json` 的调试配置，我们可以在 `build` 目录下执行此程序。

1.  进入 `build` 目录：
    ```bash
    cd build
    ```

2.  执行 SLAM 程序（此处的参数参考了 `launch.json`）：
    ```bash
    ./bathy_slam_real --simulation yes --bathy_survey ../sim_data/250826_sub_maps/NESP_noINS_shortTest/ --config ../config.yaml
    ```

    执行完毕后，`build` 目录下会生成用于比较的位姿文件，通常包括：
    *   `poses_original.txt` (原始真值)
    *   `poses_corrupted.txt` (添加噪声后的轨迹)
    *   `poses_optimized.txt` (经过图优化的轨迹)

### 第 2 步：运行 `plot_results.py` 进行可视化

程序运行结束后，我们回到项目的根目录，并调用 `plot_results.py` 脚本来可视化比较这三条轨迹。

1.  回到项目根目录：
    ```bash
    cd ..
    ```

2.  运行绘图脚本：
    ```bash
    python3 scripts/plot_results.py \
        --initial_poses build/poses_original.txt \
        --corrupted_poses build/poses_corrupted.txt \
        --optimized_poses build/poses_optimized.txt
    ```

### `.vscode/launch.json` 配置

以下是一个示例配置，展示了如何在 VSCode 中设置调试环境以便于运行 `plot_results.py` 脚本：

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Python：绘制 SLAM 结果",
            "type": "debugpy",
            "request": "launch",
            "program": "${workspaceFolder}/scripts/plot_results.py",
            "console": "integratedTerminal",
            "args": [
                "--initial_poses",
                "build/poses_original.txt",
                "--corrupted_poses",
                "build/poses_corrupted.txt",
                "--optimized_poses",
                "build/poses_optimized.txt"
            ],
            "cwd": "${workspaceFolder}"
        }
    ]
}
```

执行此命令后，将会弹出一个 3D 绘图窗口，用不同颜色的轨迹线清晰地展示 SLAM 算法的优化效果。
