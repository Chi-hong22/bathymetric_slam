## 在线定位误差评估与增量优化实施方案

### 1. 背景与目标
- 现有 `test_slam_real` 流程完全离线：构图→一次 Ceres→输出轨迹。导师要求“每次节点注入即优化，并输出以 ping 时间为横轴的定位误差曲线”。
- 输入数据固定：只有子地图关键帧姿态、点云与其覆盖的 ping（局部坐标）。缺失单 ping 真值/里程信息，因此 **真实的 per-ping 误差无法直接计算**，只能在关键帧层面近似。
- 目标：基于现有数据，模拟在线 SLAM 行为，生成“在线估计 vs 真值”的时间序列，并提供绘图接口。

### 2. 数据限制与必要假设
1. **误差聚焦在关键帧**：噪声注入与优化均作用于 `SubmapObj::submap_tf_`，所有 ping 共享同一姿态偏差。
2. **缺乏真实时间戳**：以 ping 序号 / 子地图内索引代替时间。
3. **per-ping 误差构造方式**：将关键帧误差按子地图覆盖的 ping 区间均分或线性插值，形成连续误差曲线。后续如获得原始导航轨迹再替换。

### 3. 整体实现流程
1. **扩充在线日志**
   - 现有输入是提前切好的 PCD 子地图，没有真实“在线生成过程”；因此将 `submap_id` 直接视为“伪时间”序列（等价于在线 SLAM 中的节点编号）。
   - 在 `BathySlam::runOffline()` 的主循环中，新增 `OnlineLogEntry`（包含 `submap_id, pseudo_time_idx, est_pose, gt_pose`，可选记录子地图点数以决定伪时间步长）。
   - 每次完成 GICP/闭环处理后，立即记录当前估计，与 `submaps_gt` 中同 ID 的真值对比。

2. **每次节点注入后的后端优化**
   - 触发频率由统一参数控制，默认 `frequency = 1`（每加入 1 个新节点就优化一次）。误差计算与该频率绑定：只有执行了后端优化，才会输出对应时间段的误差。
   - 在添加完第 `i` 个子地图及其相关边后调用 `triggerIncrementalOptimization(i)`：
     1. 使用当前 `GraphConstructor` 内容写 `graph_partial.g2o`。
     2. 调用 Ceres（迭代次数由参数控制，例如 `max_online_iterations`）获取前 `i` 个节点的最新估计。
     3. 用 `updateSubmapsCeres` 更新 `submaps_reg[0..i]`，并同步记录到在线日志。
   - 添加配置项控制触发频率（如 `incremental_optimize=true`, `min_nodes_before_opt=5`），避免早期图过小或计算负担过重。

3. **生成“伪 ping”误差序列**
   - 没有真实 ping ID，只能把每个子地图视为“上一关键帧至当前关键帧”的时间区间；伪时间长度由“子地图包含的 ping 数 × 固定采样周期 Δt”决定。Δt 作为绘图脚本的输入参数，由用户在运行脚本时设置。
   - 对每条日志记录，已知关键帧误差 `Δpose_i = est_pose_i - gt_pose_i`。
   - 误差构造策略：
     - **均匀分配**：该区间内所有伪时间点（即等间隔的 ping）获得相同误差。
     - **线性插值**：误差在区间内由 0 平滑增加到 `Δpose_i`，更贴近“逐步漂移”。
   - 输出 `ping_error.csv`：`time_s, err_xy, err_yaw, source_submap_id`（其中 `time_s = ping_idx * Δt`）；脚本也可保留 `pseudo_idx` 供调试。

4. **绘图脚本**
   - 新增 `scripts/plot_ping_error.py`（或扩展现有 `plot_results.py`），输入 `ping_error.csv`，绘制 `time vs error_xy`、`time vs error_yaw`。
   - 可选叠加关键帧误差阶梯线，用于说明误差分摊的近似关系。

3. **生成“伪 ping”误差序列**
   - 对每条日志记录，沿伪时间区间展开关键帧误差 `Δpose_i = est_pose_i - gt_pose_i`。
   - 误差构造策略：
     - **均匀分配**：区间内所有伪时间点赋同样误差。
     - **线性插值**：误差在区间内由 0 平滑增加到 `Δpose_i`。
   - 额外输出 **纯惯导（DR）误差**：使用 `gaussian_noise_guide.md` 中的“真值 + yaw 噪声”生成的 DR 链（`graph_obj.drChain_` / `drMeas_`）与真值比较，并按相同步长展开，作为绘图对照。
   - 输出 `ping_error.csv`：`time_s, err_xy, err_yaw, source_submap_id, err_xy_dr, err_yaw_dr`（脚本内部再根据 Δt 生成时间轴）。

4. **绘图脚本**
   - 新增 `scripts/plot_online_error.py`（命名可调整），读取 `ping_error.csv`，分别输出两张图：`time vs error_xy`、`time vs error_yaw`。
   - 每张图叠加两条曲线：`online_estimate` 与 `pure_DR`。
   - 输出目录、 DPI、是否保存等参数与 `plot_results.py` 保持一致；图片命名遵循 `plot_results.py` 的模式。
   - 原有 `scripts/plot_results.py` 在文件头部添加注释：**“仅适用于完整离线 SLAM 流程结束后的 pose 对比”**，避免混淆。

### 4. 模块改动点
| 模块 | 变更 | 目的 |
| ---- | ---- | ---- |
| `BathySlam::runOffline` | 插入在线日志记录、统计 ping 区间 | 支持时间序列输出 |
| `GraphConstructor` & `ceres_optimizer` | 提供“部分图”保存与快速求解接口；配置迭代上限 | 支持每步增量优化 |
| `SubmapsVec` 数据结构 | 记录伪时间长度（例如点数或固定步数） | 为误差分摊提供依据 |
| 新增 `OnlineLogWriter` | 统一管理 CSV/JSON 输出，避免主流程杂乱 | 后续分析 |
| 新增 `plot_online_error.py` | 生成 `time vs error_xy`、`time vs error_yaw` 并叠加 DR 曲线 | 展示结果 |
| 更新 `plot_results.py` 注释 | 标明适用范围（离线完成后使用） | 使用指引 |

### 5. 里程碑与验证
1. **阶段一：日志与增量优化**  
   - 验证每次节点注入后 Ceres 能正常运行，`online_log.csv` 有逐节点估计。
2. **阶段二：误差分摊与导出**  
   - 检查 `ping_error.csv` 是否连续、值域合理（可随机抽取子地图人工计算对比）。
3. **阶段三：绘图**  
   - 运行脚本生成曲线，确认随节点增长误差趋势与实际表现一致。

### 6. 参数暴露与 VS Code 启动配置
- **未来工作**：在检测不到任何重叠、且仍处于梳妆路径第一条直线时，仅计算误差不触发优化。实现方式：在调用 `triggerIncrementalOptimization` 前检查 `submap_i.overlaps_idx_.empty()` 并结合 swath/航程条件，若满足则直接写日志。此逻辑作为后续扩展。
- 新增 CLI 参数（示例，实际实现时需在 `cxxopts` 等解析器内添加，并在 `.vscode/launch.json` 中给出中文注释）：
  - `--online_opt_enable`：是否启用增量优化与在线日志；
  - `--online_opt_freq <int>`：触发频率/误差输出频率，默认 1；
  - `--online_opt_max_iter <int>`：每次增量 Ceres 的最大迭代数；
  - `--online_log_path <path>`：在线误差日志输出目录；
  - `--online_plot_input <path>`：供绘图脚本读取的 CSV。
- `launch.json` 里对应配置项需要附中文说明（例如“在线优化开关”“每次优化最大迭代数”），确保无需改代码即可调整。
- Python 调试配置新增一条：运行 `scripts/plot_online_error.py`（参数包括 `--ping_error_csv`, `--ping_dt`（固定 ping 时间间隔，单位秒）, `--save_fig` 等）。`plot_results.py` 保留，但在脚本头部及 `launch.json` 中明确“仅适用于离线 SLAM 完成后的误差对比”。

### 7. 里程碑与验证
1. **阶段一：日志与增量优化**  
   - 验证每次节点注入后 Ceres 能正常运行，`online_log.csv` 有逐节点估计。
2. **阶段二：误差分摊与导出**  
   - 检查 `ping_error.csv` 是否连续、值域合理（可随机抽取子地图人工计算对比）。
3. **阶段三：绘图**  
   - 运行新脚本生成 `error_xy`、`error_yaw` 曲线，并确认 DR 对照曲线存在。

### 8. 后续可扩展方向
- 获取 AUV 原始导航/IMU 数据后，可替换当前“关键帧均分”策略，实现真实 per-ping 误差。
- 引入更轻量的增量求解器（如 iSAM2）以减少每步优化耗时。
- 将在线误差反馈回系统，用于动态阈值或自动重定位策略。

> **注意**：本文方案以当前数据条件为前提，任何 per-ping 精度分析均应明确说明“误差在子地图内均匀分布/线性插值”的假设。后续若数据源增强，可在不改接口的情况下替换误差生成模块。

