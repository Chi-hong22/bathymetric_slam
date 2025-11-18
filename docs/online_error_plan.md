## 在线定位误差评估与增量优化实施方案

> 本文档用于帮助协作者快速理解“在线增量 SLAM + 时间误差评估”特性，包括设计目标、数据限制、核心模块、配置入口以及如何调试。按照本文档即可在不阅读全部代码的情况下进行定制或调优。

---

### 1. 背景
- **遗留流程**：`test_slam_real` 原为离线批处理：构图 → 添加噪声 → 保存 g2o → 一次性 Ceres → 输出轨迹。
- **新增需求**：
  1. 需要模拟“在线”行为：每加入一个子地图就进行一次后端优化（或按固定频率）。
  2. 需要输出以时间为横轴的定位误差曲线（含纯惯导 vs 在线估计）。
  3. 所有配置必须集中在 `config.yaml`，避免复杂命令行。

### 2. 数据限制与假设
- 输入仍为离线生成的 PCD 子地图集合，缺少真实 ping 时间与里程计序列。
- 每个子地图只有一个关键帧姿态，对应一段 ping 序列；我们将 `submap_id` 视作“伪时间”节点。
- per-ping 误差只能通过“将关键帧误差在该子地图内均分/插值”近似得到。

### 3. 关键配置
在 `config.yaml` 中新增以下字段（均有默认值）：
| 参数 | 作用 |
| --- | --- |
| `online_opt_enable` | 是否启用在线增量优化与误差日志 |
| `online_opt_freq` | 每加入多少个子地图触发一次增量 Ceres |
| `online_opt_max_iter` | 增量 Ceres 每次允许的最大迭代数 |
| `online_log_path` | 在线日志、临时 g2o、`ping_error.csv` 的输出目录 |
| `online_plot_input` | 绘图脚本默认读取的 CSV 路径 |

VSCode 调试配置只需指定 `--simulation / --bathy_survey / --config`，无需再塞入在线参数。

### 4. 修改概览
1. **BathySlam::runOffline**
   - 引入 `OnlineLogEntry` 缓存：记录 `submap_id`、伪时间索引、估计位姿、真值位姿、DR 位姿、子地图包含的 ping 数。
   - 在处理每个子地图时，如果 `online_opt_enable=true` 且已存在边，调用 `tryRunOnlineOptimization()`。
   - 当还没有任何 DR/LC 边（例如子图 0），直接跳过在线优化，避免空图保存/重复初始化 glog。
   - 结束后调用 `online_logger.writeRawLog()` 和 `writePingErrorCsv()` 生成 `online_log.csv` 与 `ping_error.csv`。

2. **GraphConstructor / ceres_optimizer**
   - `GraphConstructor::saveG2OFile()` 已能处理空边；`tryRunOnlineOptimization()` 在调用前再判断一次是否为空。
   - `ceres_optimizer::ceresSolver()` 支持自定义最大迭代数、可选是否导出 poses_corrupted/poses_optimized（在线模式禁用，离线保留）。

3. **OnlineLogWriter（新组件）**
   - 负责缓存在线日志，并输出：
     - `online_log.csv`: 便于调试。
     - `ping_error.csv`: 字段为 `ping_index, err_xy, err_yaw, err_xy_dr, err_yaw_dr, source_submap_id`。
   - 误差计算：`err_xy` 为 XY 平面欧式距离，`err_yaw` 为 yaw 角差，DR 同理。若子地图包含 `N` 个 ping，则将误差线性展开到 `N` 个采样点。

4. **绘图脚本**
   - 新增 `scripts/plot_online_error.py`，读取 `ping_error.csv`、结合 `--ping_dt` 生成时间轴（秒），绘制在线估计与纯 DR 的 `time vs error_xy`、`time vs error_yaw`。
   - `scripts/plot_results.py` 增加说明：仅适用于离线流程的轨迹对比。

- README / 其他
  - README 的“在线增量版本”章节说明配置项与绘图流程。
  - `.vscode/launch.json` 提供新的 Python debug 配置以运行 `plot_online_error.py`。

### 5. 在线优化流程详解
1. **触发时机**：每当 `online_opt_enable=true` 且 `(submaps_reg.size() % online_opt_freq == 0)`，并且图中已有至少一条边。
2. **流程**：
   - `GraphConstructor::saveG2OFile("build/graph_online_tmp.g2o")`
   - `ceres_solver("--graph_online_tmp.g2o", dr_edge_count, online_opt_max_iter, export_debug=false)`
   - `updateSubmapsCeres(poses, submaps_reg)`
3. **glog 初始化**：统一在 `main()` 中调用一次 `google::InitGoogleLogging(argv[0]);`，在线/离线都复用，避免 “Init twice” 崩溃。

### 6. 误差生成逻辑
1. `pseudo_time_idx`：累计“子地图包含的 ping 数”。
2. `ping_index = pseudo_time_idx + k`（k 从 0 到 ping_count-1），绘图脚本乘以 `--ping_dt` 得时间。
3. `err_xy` / `err_yaw`：在一个子地图内按线性比例递增（模拟误差随时间积累）。DR 误差来自在 `BathySlam::runOffline()` 里创建 DR 边后立即调用 `graph_obj_->addNoiseToLastDREdge()` 累积出的噪声链，因此能够真实反映“惯导发散”。

### 7. 输出文件
| 文件 | 说明 |
| --- | --- |
| `build/online_log.csv` | 每个子地图的日志（伪时间、姿态等） |
| `build/ping_error.csv` | 供绘图脚本使用的误差序列 |
| `build/graph_online_tmp.g2o` | 在线 Ceres 的临时 g2o（每次触发时覆盖） |

### 8. 调试建议
1. 确认 `config.yaml` 中在线参数设置正确；若 `online_opt_enable=false`，运行行为等同原始版本。
2. 调整 `online_opt_freq` 可降低增量优化频率。例如设置为 5 表示每 5 个子地图优化一次。
3. `online_opt_max_iter` 过大可能导致在线阶段耗时；建议视数据情况在 10–50 之间调节。
4. `--ping_dt` 需要由数据集提供（ping 间隔时间），默认脚本为 1 秒，可在调试配置中修改。

### 9. 后续扩展
- **前段首条直线的“只记录不优化”**：可在 `BathySlam::runOffline` 中检测 `submap_i.overlaps_idx_.empty()` + “仍处于首条直线路段”时仅日志不触发优化。
- **真实 per-ping 轨迹**：若未来获取原始导航数据，可替换 `ping_error.csv` 的生成策略，以真实时间戳取代伪时间。
- **更轻量的增量优化器**：若在线 Ceres 仍过重，可考虑集成 iSAM2 或 g2o incremental。


