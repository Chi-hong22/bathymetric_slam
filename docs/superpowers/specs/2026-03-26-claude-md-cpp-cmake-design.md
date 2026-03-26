# 2026-03-26 CLAUDE.md C++/CMake 补充设计

## 背景
现有仓库级 `CLAUDE.md` 已覆盖构建/运行、架构概览、配置与路径约束、验证方式，但对 C++ 与 CMake 的结构性事实还不够聚焦。用户要求补充这部分内容，并明确以 `src/CMakeLists.txt` 为主，同时保持整体文档简洁，不扩展为完整构建手册。

## 目标
在不显著增加文档体积的前提下，让未来的 Claude Code 实例快速掌握本仓库的 C++/CMake 关键约束：
- 核心构建入口以 `src/CMakeLists.txt` 为主
- 当前 C++ 编译基线与编译选项
- 依赖发现方式
- 一级模块的 CMake 组织方式
- 应用目标与运行产物位置

## 约束
- 保持精简，避免新增冗长子章节
- 不引入不存在的 lint/test 规则
- 不把 README/CMake 内容逐字复述到 `CLAUDE.md`
- 不写未经源码证实的构建行为

## 方案比较

### 方案 A：新增精简的 `C++ and CMake structure` 章节（推荐）
在现有 `CLAUDE.md` 中新增一个短章节，集中放 C++/CMake 事实。

**优点**
- 信息集中，未来检索成本低
- 不打散现有 build/run 与 architecture 内容
- 能清楚体现“以 `src/CMakeLists.txt` 为主”

**缺点**
- 文档略有增长

### 方案 B：把 C++/CMake 内容分散揉进现有章节
不新增章节，只在 `Build and run` 与 `Architecture overview` 中追加若干 bullet。

**优点**
- 文字更少

**缺点**
- CMake 结构信息不集中
- 后续维护时更容易重复或遗漏

## 采用方案
采用 **方案 A**，但章节控制在少量 bullet 内，保持“稍微精简”。

## 拟写入内容
新增章节建议命名为：

`## C++ and CMake structure`

仅包含以下要点：

1. `src/CMakeLists.txt` 是仓库主要构建入口，应优先以它理解编译标准、依赖和模块边界。
2. 当前构建优先使用 C++17，并在该文件中设置了 `-O3 -fPIC`，同时通过 `add_compile_options(... -Wextra)` 增加编译告警。
3. 依赖通过 `find_package(...)` 发现；除非用户明确要求，不要改成 vendored、FetchContent 或本地绝对路径依赖方案。
4. 一级模块由 `add_subdirectory(...)` 组织：`apps`、`graph_optimization`、`registration`、`submaps_tools`、`bathy_slam`、`meas_models`。
5. `src/apps/CMakeLists.txt` 定义 `bathy_slam_real` / `read_auv_data`，并把运行产物输出到 `bin/`。
6. 如果修改目标名、链接关系、输出目录或 build type，需同时复查运行路径与 `.vscode/launch.json`。

## 不写入内容
为保持精简，本次不写入：
- 每个 `find_package(...)` 依赖的完整列表
- 所有 target_link_libraries 细节
- 不存在的 `ctest`/lint 规则
- 过细的 CMake 变量背景说明

## 变更影响
该补充不会改变现有 `CLAUDE.md` 的主结构，只会新增一个短章节，并与现有的 `Build and run` / `Architecture overview` 形成互补：
- `Build and run`：怎么构建、怎么运行
- `C++ and CMake structure`：构建系统是如何组织的
- `Architecture overview`：代码职责如何分层

## 验收标准
更新后的 `CLAUDE.md` 应满足：
1. 明确点名 `src/CMakeLists.txt` 是 C++/CMake 事实主来源
2. 包含 C++17、`find_package(...)`、`add_subdirectory(...)`、`bin/` 输出这几项核心信息
3. 没有与现有章节产生明显重复
4. 整体长度只小幅增加，仍适合作为未来实例快速阅读的入口文档
