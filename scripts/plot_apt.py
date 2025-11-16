#!/usr/bin/env python3
"""
绘制轨迹的 Absolute Position Trajectory (APT) 时间序列脚本

功能概述：
1. 加载估计轨迹与真值轨迹
2. 根据时间戳（或索引）建立对应点
3. 计算对应点之间的欧式位置距离
4. 绘制距离随时间变化的曲线，并叠加一条拟合曲线

使用示例：
  python3 scripts/plot_apt.py --estimated build/poses_optimized.txt --ground_truth build/poses_original.txt --verbose

说明：
脚本仅输出APT位置距离的时间序列图，不进行对齐或额外统计计算。
"""

import matplotlib.pyplot as plt
import numpy as np
import sys
import os
from datetime import datetime
from optparse import OptionParser
import warnings

# 忽略一些常见的数值警告
warnings.filterwarnings('ignore', category=RuntimeWarning)
warnings.filterwarnings('ignore', category=UserWarning, message='.*missing from current font.*')

# 设置中文字体支持（如果可用）
try:
    plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'SimHei', 'Arial Unicode MS']
    plt.rcParams['axes.unicode_minus'] = False
except Exception:
    pass


def create_parser():
    """创建命令行参数解析器"""
    parser = OptionParser(usage="python plot_apt.py --estimated <file> --ground_truth <file> [options]")

    parser.add_option("--estimated",
                      dest="estimated_poses",
                      default="",
                      help="估计轨迹文件路径 (必需)")
    parser.add_option("--ground_truth",
                      dest="ground_truth_poses",
                      default="",
                      help="真值轨迹文件路径 (必需)")
    parser.add_option("--max_pairs",
                      dest="max_pairs",
                      type="int",
                      default=10000,
                      help="用于APT分析的最大位姿对数 (默认: 10000)")
    parser.add_option("--output_dir",
                      dest="output_dir",
                      default="build/apt_analysis",
                      help="输出目录 (默认: build/apt_analysis)")
    parser.add_option("--prefix",
                      dest="output_prefix",
                      default="apt",
                      help="输出文件前缀 (默认: apt)")
    parser.add_option("--dpi",
                      dest="dpi",
                      type="int",
                      default=300,
                      help="图像分辨率 (默认: 300)")
    parser.add_option("--save_fig",
                      dest="save_fig",
                      default="",
                      help="保存图像的路径或目录，若未提供则使用 --output_dir")
    parser.add_option("--fit_method",
                      dest="fit_method",
                      default="local_poly",
                      help="拟合方式: 'poly' 全局多项式，'local_poly' 局部多项式 (默认: local_poly)")
    parser.add_option("--fit_degree",
                      dest="fit_degree",
                      type="int",
                      default=3,
                      help="拟合多项式阶数 (默认: 3)")
    parser.add_option("--fit_window",
                      dest="fit_window",
                      type="int",
                      default=15,
                      help="局部多项式拟合窗口大小，需为奇数 (默认: 15)")
    parser.add_option("--no_show",
                      action="store_true",
                      dest="no_show",
                      default=False,
                      help="不显示图像窗口")
    parser.add_option("--verbose",
                      action="store_true",
                      dest="verbose",
                      default=False,
                      help="详细输出")

    return parser


def load_trajectory_data(file_path, verbose=False):
    """
    加载轨迹数据
    支持格式：timestamp x y z [qx qy qz qw] 或仅 x y z
    返回: (timestamps, positions, has_timestamps)
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"轨迹文件不存在: {file_path}")

    try:
        data = np.genfromtxt(file_path, comments='#')

        if data.size == 0:
            raise ValueError(f"文件为空: {file_path}")

        if data.ndim == 1:
            data = data.reshape(1, -1)

        n_cols = data.shape[1]

        if verbose:
            print(f"加载轨迹文件: {file_path}")
            print(f"  数据形状: {data.shape}")
            print(f"  列数: {n_cols}")

        if n_cols >= 4:
            timestamps = data[:, 0]
            positions = data[:, 1:4]
            has_timestamps = True
        elif n_cols == 3:
            timestamps = np.arange(len(data))
            positions = data[:, :3]
            has_timestamps = False
            if verbose:
                print("  未检测到时间戳列，使用序号作为时间戳")
        else:
            raise ValueError(f"不支持的数据格式，列数: {n_cols}，期望至少3列")

        if verbose:
            print(f"  加载位姿数: {len(positions)}")
            print(f"  时间范围: {timestamps[0]:.3f} - {timestamps[-1]:.3f}")
            print(f"  位置范围: X[{positions[:, 0].min():.3f}, {positions[:, 0].max():.3f}] "
                  f"Y[{positions[:, 1].min():.3f}, {positions[:, 1].max():.3f}] "
                  f"Z[{positions[:, 2].min():.3f}, {positions[:, 2].max():.3f}]")

        return timestamps, positions, has_timestamps

    except Exception as e:
        raise RuntimeError(f"加载轨迹文件失败 {file_path}: {str(e)}")


def associate_trajectories(timestamps_est, positions_est, timestamps_gt, positions_gt,
                           max_time_diff=0.02, verbose=False):
    """
    根据时间戳关联两条轨迹
    返回: (associated_est, associated_gt, valid_timestamps)
    """
    if verbose:
        print(f"关联轨迹，最大时间差阈值: {max_time_diff}s")

    associated_est = []
    associated_gt = []
    valid_timestamps = []

    for i, t_est in enumerate(timestamps_est):
        time_diffs = np.abs(timestamps_gt - t_est)
        min_idx = np.argmin(time_diffs)
        min_diff = time_diffs[min_idx]

        if min_diff <= max_time_diff:
            associated_est.append(positions_est[i])
            associated_gt.append(positions_gt[min_idx])
            valid_timestamps.append(t_est)

    if len(associated_est) == 0:
        raise ValueError(f"无法关联轨迹，时间差阈值过小: {max_time_diff}s")

    associated_est = np.array(associated_est)
    associated_gt = np.array(associated_gt)
    valid_timestamps = np.array(valid_timestamps)

    if verbose:
        print(f"  成功关联位姿对数: {len(associated_est)}")
        print(f"  估计轨迹利用率: {len(associated_est)}/{len(positions_est)} "
              f"({100 * len(associated_est) / len(positions_est):.1f}%)")
        print(f"  真值轨迹利用率: {len(associated_est)}/{len(positions_gt)} "
              f"({100 * len(associated_est) / len(positions_gt):.1f}%)")

    return associated_est, associated_gt, valid_timestamps


def compute_position_distances(estimated, ground_truth):
    """计算对应点的欧式位置距离"""
    if estimated.shape != ground_truth.shape:
        raise ValueError("估计轨迹与真值轨迹的形状不一致")
    return np.linalg.norm(estimated - ground_truth, axis=1)


def fit_distance_curve(timestamps, distances, options):
    """根据选项拟合距离曲线，返回拟合后的数值；若样本不足则返回None"""
    if len(distances) < 2:
        return None

    method = options.fit_method.lower()
    degree = max(1, options.fit_degree)

    timestamps = np.asarray(timestamps, dtype=float)
    distances = np.asarray(distances, dtype=float)

    if method == "poly":
        max_degree = len(distances) - 1
        if max_degree < 1:
            return None
        degree = min(degree, max_degree)
        t_mean = timestamps.mean()
        t_centered = timestamps - t_mean
        coeffs = np.polyfit(t_centered, distances, degree)
        return np.polyval(coeffs, t_centered)

    if method == "local_poly":
        window = max(options.fit_window, degree + 1)
        if window % 2 == 0:
            window += 1
        window = min(window, len(distances) if len(distances) % 2 == 1 else len(distances) - 1)
        if window < degree + 1:
            window = degree + 1
            if window % 2 == 0:
                window += 1
            window = min(window, len(distances))
            if window < degree + 1:
                return distances.copy()
        return local_polynomial_smoothing(timestamps, distances, window, degree)

    raise ValueError(f"未知的拟合方式: {options.fit_method}")


def local_polynomial_smoothing(timestamps, distances, window_size, degree):
    """简化版局部多项式平滑，实现类似 LOESS 的拟合效果"""
    if window_size < degree + 1:
        raise ValueError("窗口大小必须大于多项式阶数")
    if window_size <= 1 or len(distances) < window_size:
        return distances.copy()

    half = window_size // 2
    padded_times = np.pad(timestamps, (half, half), mode='edge')
    padded_distances = np.pad(distances, (half, half), mode='edge')

    smoothed = np.empty_like(distances, dtype=float)

    for i in range(len(distances)):
        local_t = padded_times[i:i + window_size]
        local_d = padded_distances[i:i + window_size]
        t0 = local_t[window_size // 2]
        shifted_t = local_t - t0

        try:
            coeffs = np.polyfit(shifted_t, local_d, min(degree, window_size - 1))
            smoothed[i] = np.polyval(coeffs, 0.0)
        except np.linalg.LinAlgError:
            smoothed[i] = local_d.mean()

    return smoothed


def resolve_save_path(options, timestamp):
    """
    根据选项生成保存路径。
    优先使用 --save_fig；若未提供，则落入 --output_dir。
    """
    if options.save_fig:
        save_path = options.save_fig
        # 判断是否为目录：存在的目录，或缺少扩展名且不以 '.' 结尾
        is_dir_like = (
            os.path.isdir(save_path) or
            (not os.path.splitext(save_path)[1] and not save_path.endswith('.'))
        )

        if is_dir_like:
            os.makedirs(save_path, exist_ok=True)
            filename = f"{options.output_prefix}_position_timeseries_{timestamp}.png"
            return os.path.join(save_path, filename)

        # 视为文件路径，构造带时间戳的文件名
        dir_part = os.path.dirname(save_path)
        base_name = os.path.basename(save_path)
        name_part, ext_part = os.path.splitext(base_name)
        if not ext_part:
            ext_part = ".png"

        suffix = "timeseries"
        new_filename = f"{name_part}_{suffix}_{timestamp}{ext_part}"

        if dir_part:
            os.makedirs(dir_part, exist_ok=True)
        else:
            dir_part = os.getcwd()

        return os.path.join(dir_part, new_filename)

    # 默认使用 output_dir
    os.makedirs(options.output_dir, exist_ok=True)
    filename = f"{options.output_prefix}_position_timeseries_{timestamp}.png"
    return os.path.join(options.output_dir, filename)


def plot_apt_timeseries(timestamps, distances, fitted, xlabel, options):
    """绘制APT位置距离时间序列图"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    fig, ax = plt.subplots(figsize=(12, 6))
    ax.plot(timestamps, distances, '-', color='red', linewidth=1.2,
            alpha=0.8, label='APT Position Distance')

    if fitted is not None:
        ax.plot(timestamps, fitted, color='blue', linewidth=2,
                linestyle='--', label='Fitted Curve')

    ax.set_xlabel(xlabel)
    ax.set_ylabel('APT Position Distance (m)')
    ax.set_title('APT Position Distance Time Series')
    ax.grid(True, alpha=0.3)
    ax.legend()

    timeseries_path = resolve_save_path(options, timestamp)
    fig.savefig(timeseries_path, dpi=options.dpi, bbox_inches='tight')
    print(f"保存APT位置距离时间序列图: {timeseries_path}")

    if not options.no_show:
        plt.show()
    else:
        plt.close(fig)


def main():
    parser = create_parser()
    (options, args) = parser.parse_args()

    if not options.estimated_poses or not options.ground_truth_poses:
        print("错误: 必须提供 --estimated 和 --ground_truth 参数")
        parser.print_help()
        sys.exit(1)

    if options.verbose:
        print("=== APT 时间序列分析开始 ===")
        print(f"估计轨迹: {options.estimated_poses}")
        print(f"真值轨迹: {options.ground_truth_poses}")
        print(f"输出目录: {options.output_dir}")

    try:
        if options.verbose:
            print("\n--- 加载轨迹数据 ---")

        timestamps_est, positions_est, has_ts_est = load_trajectory_data(
            options.estimated_poses, options.verbose)
        timestamps_gt, positions_gt, has_ts_gt = load_trajectory_data(
            options.ground_truth_poses, options.verbose)

        if options.verbose:
            print("\n--- 轨迹时间戳关联 ---")

        if has_ts_est and has_ts_gt:
            associated_est, associated_gt, valid_timestamps = associate_trajectories(
                timestamps_est, positions_est, timestamps_gt, positions_gt, verbose=options.verbose)
            xlabel = 'Timestamp'
        elif not has_ts_est and not has_ts_gt:
            min_len = min(len(positions_est), len(positions_gt))
            associated_est = positions_est[:min_len]
            associated_gt = positions_gt[:min_len]
            valid_timestamps = np.arange(min_len)
            xlabel = 'Index'
            if options.verbose:
                print(f"  按序号对齐，使用前 {min_len} 个位姿")
        else:
            raise ValueError("估计轨迹与真值轨迹的时间戳格式不一致")

        if len(associated_est) > options.max_pairs:
            indices = np.linspace(0, len(associated_est) - 1, options.max_pairs, dtype=int)
            associated_est = associated_est[indices]
            associated_gt = associated_gt[indices]
            valid_timestamps = valid_timestamps[indices]
            if options.verbose:
                print(f"  限制APT分析位姿对数为: {options.max_pairs}")

        distances = compute_position_distances(associated_est, associated_gt)
        fitted = fit_distance_curve(valid_timestamps, distances, options)

        plot_apt_timeseries(valid_timestamps, distances, fitted, xlabel, options)

        if options.verbose:
            print(f"\n=== APT 时间序列分析完成，结果保存至: {options.output_dir} ===")

    except Exception as e:
        print(f"错误: {str(e)}")
        if options.verbose:
            import traceback
            traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()

