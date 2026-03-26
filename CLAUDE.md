# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build and run

- Standard build flow comes from [README.md](README.md):

  ```bash
  mkdir -p build
  cd build
  cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install ..
  make -j4
  make install
  ```
- G2O is expected to be installed system-wide. Both [README.md](README.md) and [.github/copilot-instructions.md](.github/copilot-instructions.md) call this out.
- The main executable is `bathy_slam_real`, built in [src/apps/CMakeLists.txt](src/apps/CMakeLists.txt). `read_auv_data` also builds there, but the repository does not document its CLI usage; default to working around `bathy_slam_real` unless the task explicitly targets `read_auv_data`.
- For local runs, treat [.vscode/launch.json](.vscode/launch.json) as the best description of the current working workflow: the program is launched from `build/` and uses relative paths such as `../config.yaml` and `../sim_data/...`.
- Common run commands:
  - Simulated submaps:

    ```bash
    ./bathy_slam_real --simulation yes --bathy_survey ../sim_data/map_small/ --config ../config.yaml
    ```
  - Real survey cereal input:

    ```bash
    ./bathy_slam_real --simulation no --bathy_survey /path/to/mbes_pings.cereal --config config.yaml
    ```
- Common post-run analysis commands:

  ```bash
  python3 scripts/plot_results.py --initial_poses build/poses_original.txt --corrupted_poses build/poses_corrupted.txt --optimized_poses build/poses_optimized.txt
  python3 scripts/plot_apt.py --estimated build/poses_optimized.txt --ground_truth build/poses_original.txt --verbose
  python3 scripts/plot_ate.py --estimated build/poses_optimized.txt --ground_truth build/poses_original.txt --alignment_type se3 --verbose
  ```

## C++ and CMake structure

- Use [src/CMakeLists.txt](src/CMakeLists.txt) as the primary source of truth for compiler settings, dependency discovery, and top-level module layout.
- The project prefers C++17 and currently sets `-O3 -fPIC`, with additional warnings enabled through `add_compile_options(... -Wextra)`.
- Keep the current dependency strategy based on `find_package(...)`; do not switch to vendored dependencies, FetchContent, or local absolute-path setups unless explicitly requested.
- Top-level modules are wired through `add_subdirectory(...)`: `apps`, `graph_optimization`, `registration`, `submaps_tools`, `bathy_slam`, and `meas_models`.
- [src/apps/CMakeLists.txt](src/apps/CMakeLists.txt) defines the app targets and sends runtime binaries to `bin/`; if you change target names, link dependencies, output paths, or build-type-related behavior, also re-check [.vscode/launch.json](.vscode/launch.json).

## Architecture overview

This repository is a submap-based offline bathymetric SLAM pipeline. The main flow is orchestrated in [src/apps/src/test_slam_real.cpp](src/apps/src/test_slam_real.cpp): it reads simulated submaps or real survey data, builds `SubmapsVec`, performs GICP registration on overlapping submaps, constructs a pose graph, exports `.g2o`, and then optimizes poses with Ceres before writing updated submaps/results.

High-level module boundaries:

- [src/apps/](src/apps/) — command-line entrypoints and end-to-end orchestration.
- [src/bathy_slam/](src/bathy_slam/) — offline SLAM loop that ties overlap detection, registration, and loop-closure insertion together.
- [src/submaps_tools/](src/submaps_tools/) — core submap data structures (`SubmapObj`, `SubmapsVec`), submap creation, overlap detection, and point/track conversions.
- [src/registration/](src/registration/) — GICP registration and target-submap construction.
- [src/graph_optimization/](src/graph_optimization/) — graph vertices/DR edges/LC edges, `.g2o` export, and Ceres optimization interfaces.
- [src/meas_models/](src/meas_models/) — measurement-model support code.

`SubmapObj` / `SubmapsVec` are the core data units that cross module boundaries. When changing data flow or map-processing logic, trace how those objects move through `submaps_tools -> registration/bathy_slam -> graph_optimization -> result serialization` before editing.

## Configuration and data-path assumptions

- [config.yaml](config.yaml) is the main algorithm/configuration entrypoint. It controls submap size, voxel downsampling, overlap threshold, GICP settings, DR noise, and loop-closure covariance strategy.
- Runtime behavior is sensitive to the current working directory because the active debug workflow uses relative paths for both config and input data.
- When reproducing issues or changing runtime behavior, verify all of the following together:
  - executable location
  - current working directory
  - relative path to `config.yaml`
  - relative path to `sim_data/` or survey input
- If you change output filenames, config paths, or runtime arguments, also review:
  - [.vscode/launch.json](.vscode/launch.json)
  - [docs/plot_results_guide.md](docs/plot_results_guide.md)
  - [docs/plot_ate_guide.md](docs/plot_ate_guide.md)
- If documentation disagrees with scripts or source defaults, prefer the actual behavior in source/scripts.
- [src/apps/CMakeLists.txt](src/apps/CMakeLists.txt) sets the runtime output directory to `bin/`; keep that in mind when diagnosing missing executable/path issues.

## Validation workflow

- The repository does not currently expose a clear `ctest`/`gtest` workflow. Do not assume automated unit-test commands exist.
- Default validation should be based on the real executable workflow:
  1. build successfully;
  2. run `bathy_slam_real` on the intended simulated or real dataset;
  3. confirm expected pose outputs exist, especially:
     - `build/poses_original.txt`
     - `build/poses_corrupted.txt`
     - `build/poses_optimized.txt`
  4. verify that the Python analysis scripts can consume those outputs.
- For parameter tuning work, prefer validating first on simulated data before moving to real survey data, which matches the guidance in [.github/copilot-instructions.md](.github/copilot-instructions.md).
- When touching architecture-sensitive code, use these files as the primary truth sources:
  - [README.md](README.md)
  - [src/apps/src/test_slam_real.cpp](src/apps/src/test_slam_real.cpp)
  - [src/CMakeLists.txt](src/CMakeLists.txt)
  - [src/apps/CMakeLists.txt](src/apps/CMakeLists.txt)
  - [config.yaml](config.yaml)
  - [.vscode/launch.json](.vscode/launch.json)
  - [.github/copilot-instructions.md](.github/copilot-instructions.md)
