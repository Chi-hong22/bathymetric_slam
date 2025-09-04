#!/usr/bin/env python3
"""
绘制轨迹的 Absolute Trajectory Error (ATE) 分析脚本

该脚本用于：
1. 加载估计轨迹与真值轨迹
2. 通过刚体/相似变换对齐轨迹
3. 计算ATE指标（RMSE、mean、median、std、max）
4. 绘制轨迹对比图、误差时间序列、误差分布直方图
5. 保存数值摘要与图像结果

支持数据格式：
- 带时间戳：timestamp x y z [qx qy qz qw]
- 仅位置：x y z

使用示例：
  python3 scripts/plot_ate.py --estimated build/poses_optimized.txt --ground_truth build/poses_original.txt --verbose
  python3 scripts/plot_ate.py --estimated traj_est.txt --ground_truth traj_gt.txt --alignment_type sim3 --plot_3d
  python3 scripts/plot_ate.py --estimated traj_est.txt --ground_truth traj_gt.txt --output_dir results --prefix my_ate

作者：基于 plot_results.py 扩展
"""

import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import sys
import os
import json
import csv
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
except:
    pass

def create_parser():
    """创建命令行参数解析器"""
    parser = OptionParser(usage="python plot_ate.py --estimated <file> --ground_truth <file> [options]")
    
    # 必需参数
    parser.add_option("--estimated",
                      dest="estimated_poses",
                      default="",
                      help="估计轨迹文件路径 (必需)")
    parser.add_option("--ground_truth",
                      dest="ground_truth_poses", 
                      default="",
                      help="真值轨迹文件路径 (必需)")
    
    # 对齐参数
    parser.add_option("--alignment_type",
                      dest="alignment_type",
                      default="se3",
                      help="对齐类型: 'se2', 'se3', 'sim3' (默认: se3)")
    parser.add_option("--max_pairs",
                      dest="max_pairs",
                      type="int",
                      default=10000,
                      help="用于对齐的最大位姿对数 (默认: 10000)")
    
    # 输出控制
    parser.add_option("--output_dir",
                      dest="output_dir",
                      default="build/ate_analysis",
                      help="输出目录 (默认: build/ate_analysis)")
    parser.add_option("--prefix",
                      dest="output_prefix",
                      default="ate",
                      help="输出文件前缀 (默认: ate)")
    parser.add_option("--dpi",
                      dest="dpi",
                      type="int",
                      default=300,
                      help="图像分辨率 (默认: 300)")
    parser.add_option("--no_show",
                      action="store_true",
                      dest="no_show",
                      default=False,
                      help="不显示图像窗口")
    
    # 绘图控制
    parser.add_option("--plot_3d",
                      action="store_true",
                      dest="plot_3d",
                      default=False,
                      help="绘制3D轨迹图（默认2D俯视图）")
    parser.add_option("--save_aligned_traj",
                      action="store_true",
                      dest="save_aligned_traj",
                      default=False,
                      help="保存对齐后的轨迹数据")
    
    # 调试与日志
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
        # 尝试读取所有列
        data = np.genfromtxt(file_path, comments='#')
        
        if data.size == 0:
            raise ValueError(f"文件为空: {file_path}")
        
        # 确保是2D数组
        if data.ndim == 1:
            data = data.reshape(1, -1)
        
        n_cols = data.shape[1]
        
        if verbose:
            print(f"加载轨迹文件: {file_path}")
            print(f"  数据形状: {data.shape}")
            print(f"  列数: {n_cols}")
        
        if n_cols >= 4:
            # 包含时间戳: timestamp x y z [...]
            timestamps = data[:, 0]
            positions = data[:, 1:4]  # x, y, z
            has_timestamps = True
        elif n_cols == 3:
            # 仅位置: x y z
            timestamps = np.arange(len(data))  # 使用序号作为时间戳
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
        # 找到最近的真值时间戳
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
        print(f"  估计轨迹利用率: {len(associated_est)}/{len(positions_est)} ({100*len(associated_est)/len(positions_est):.1f}%)")
        print(f"  真值轨迹利用率: {len(associated_est)}/{len(positions_gt)} ({100*len(associated_est)/len(positions_gt):.1f}%)")
    
    return associated_est, associated_gt, valid_timestamps

def umeyama_alignment(X, Y, with_scale=False):
    """
    Umeyama 算法实现轨迹对齐
    X: 估计轨迹 (N x 3)
    Y: 真值轨迹 (N x 3) 
    with_scale: 是否包含尺度变换 (SE3 vs Sim3)
    
    返回: (R, t, s) 使得 Y ≈ s * R @ X + t
    """
    assert X.shape == Y.shape
    n, m = X.shape
    
    # 计算质心
    mu_X = X.mean(axis=0)
    mu_Y = Y.mean(axis=0)
    
    # 去质心
    X_centered = X - mu_X
    Y_centered = Y - mu_Y
    
    # 计算协方差矩阵
    H = X_centered.T @ Y_centered
    
    # SVD分解
    U, S, Vt = np.linalg.svd(H)
    
    # 计算旋转矩阵
    R = Vt.T @ U.T
    
    # 确保是正确的旋转矩阵（行列式为正）
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    
    # 计算尺度
    if with_scale:
        var_X = np.var(X_centered, axis=0).sum()
        if var_X > 1e-12:
            s = np.trace(np.diag(S)) / var_X
        else:
            s = 1.0
    else:
        s = 1.0
    
    # 计算平移
    t = mu_Y - s * R @ mu_X
    
    return R, t, s

def compute_ate_metrics(errors):
    """计算ATE指标"""
    errors_norm = np.linalg.norm(errors, axis=1)
    
    metrics = {
        'rmse': np.sqrt(np.mean(errors_norm**2)),
        'mean': np.mean(errors_norm),
        'median': np.median(errors_norm),
        'std': np.std(errors_norm),
        'max': np.max(errors_norm),
        'min': np.min(errors_norm)
    }
    
    return metrics, errors_norm

def plot_ate_results(original_est, aligned_est, ground_truth, errors_norm, timestamps, metrics, options):
    """绘制ATE分析结果"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # 1. 轨迹对比图
    fig1 = plt.figure(figsize=(12, 8))
    
    if options.plot_3d:
        ax = fig1.add_subplot(111, projection='3d')
        ax.plot(ground_truth[:, 0], ground_truth[:, 1], ground_truth[:, 2], 
                '-', color='green', linewidth=2, alpha=0.8, label='Ground Truth')
        ax.plot(original_est[:, 0], original_est[:, 1], original_est[:, 2], 
                '--', color='red', linewidth=1.5, alpha=0.7, label='Estimated (Original)')
        ax.plot(aligned_est[:, 0], aligned_est[:, 1], aligned_est[:, 2], 
                '-', color='blue', linewidth=1.5, alpha=0.8, label='Estimated (Aligned)')
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'Trajectory Comparison (3D) - ATE RMSE: {metrics["rmse"]:.4f} m')
    else:
        ax = fig1.add_subplot(111)
        ax.plot(ground_truth[:, 0], ground_truth[:, 1], 
                '-', color='green', linewidth=2, alpha=0.8, label='Ground Truth')
        ax.plot(original_est[:, 0], original_est[:, 1], 
                '--', color='red', linewidth=1.5, alpha=0.7, label='Estimated (Original)')
        ax.plot(aligned_est[:, 0], aligned_est[:, 1], 
                '-', color='blue', linewidth=1.5, alpha=0.8, label='Estimated (Aligned)')
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f'Trajectory Comparison (Top View) - ATE RMSE: {metrics["rmse"]:.4f} m')
        ax.set_aspect('equal', adjustable='box')
        ax.grid(True, alpha=0.3)
    
    ax.legend()
    
    # 保存轨迹对比图
    view_suffix = "3d" if options.plot_3d else "2d"
    traj_filename = f"{options.output_prefix}_trajectories_{view_suffix}_{timestamp}.png"
    traj_path = os.path.join(options.output_dir, traj_filename)
    fig1.savefig(traj_path, dpi=options.dpi, bbox_inches='tight')
    print(f"保存轨迹对比图: {traj_path}")
    
    # 2. ATE误差时间序列图
    fig2, ax2 = plt.subplots(figsize=(12, 6))
    ax2.plot(timestamps, errors_norm, '-', color='red', linewidth=1, alpha=0.7)
    ax2.axhline(y=metrics['rmse'], color='blue', linestyle='--', linewidth=2, 
                label=f'RMSE: {metrics["rmse"]:.4f} m')
    ax2.axhline(y=metrics['mean'], color='orange', linestyle='--', linewidth=2,
                label=f'Mean: {metrics["mean"]:.4f} m')
    ax2.axhline(y=metrics['median'], color='green', linestyle='--', linewidth=2,
                label=f'Median: {metrics["median"]:.4f} m')
    
    ax2.set_xlabel('Timestamp')
    ax2.set_ylabel('ATE (m)')
    ax2.set_title('ATE Error Time Series')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    # 保存误差时间序列图
    timeseries_filename = f"{options.output_prefix}_errors_timeseries_{timestamp}.png"
    timeseries_path = os.path.join(options.output_dir, timeseries_filename)
    fig2.savefig(timeseries_path, dpi=options.dpi, bbox_inches='tight')
    print(f"保存误差时间序列图: {timeseries_path}")
    
    # 3. ATE误差直方图
    fig3, ax3 = plt.subplots(figsize=(10, 6))
    n_bins = min(50, len(errors_norm) // 10)  # 自适应bin数量
    ax3.hist(errors_norm, bins=n_bins, alpha=0.7, color='skyblue', edgecolor='black')
    ax3.axvline(x=metrics['rmse'], color='blue', linestyle='--', linewidth=2,
                label=f'RMSE: {metrics["rmse"]:.4f} m')
    ax3.axvline(x=metrics['mean'], color='orange', linestyle='--', linewidth=2,
                label=f'Mean: {metrics["mean"]:.4f} m')
    ax3.axvline(x=metrics['median'], color='green', linestyle='--', linewidth=2,
                label=f'Median: {metrics["median"]:.4f} m')
    
    ax3.set_xlabel('ATE (m)')
    ax3.set_ylabel('Frequency')
    ax3.set_title('ATE Error Distribution Histogram')
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    
    # 保存误差直方图
    hist_filename = f"{options.output_prefix}_errors_histogram_{timestamp}.png"
    hist_path = os.path.join(options.output_dir, hist_filename)
    fig3.savefig(hist_path, dpi=options.dpi, bbox_inches='tight')
    print(f"保存误差直方图: {hist_path}")
    
    # 显示控制
    if not options.no_show:
        plt.show()
    else:
        plt.close('all')

def save_ate_metrics(metrics, errors_norm, timestamps, options):
    """保存ATE数值结果"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # 保存JSON摘要
    json_filename = f"{options.output_prefix}_metrics_{timestamp}.json"
    json_path = os.path.join(options.output_dir, json_filename)
    
    summary = {
        'timestamp': timestamp,
        'alignment_type': options.alignment_type,
        'num_poses': len(errors_norm),
        'metrics': metrics,
        'statistics': {
            'percentile_25': np.percentile(errors_norm, 25),
            'percentile_75': np.percentile(errors_norm, 75),
            'percentile_95': np.percentile(errors_norm, 95),
            'percentile_99': np.percentile(errors_norm, 99)
        }
    }
    
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    print(f"保存JSON摘要: {json_path}")
    
    # 保存CSV详细数据
    csv_filename = f"{options.output_prefix}_detailed_{timestamp}.csv"
    csv_path = os.path.join(options.output_dir, csv_filename)
    
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'ate_error'])
        for t, e in zip(timestamps, errors_norm):
            writer.writerow([t, e])
    print(f"保存CSV详细数据: {csv_path}")

def save_aligned_trajectories(original_est, aligned_est, ground_truth, timestamps, R, t, s, options):
    """保存对齐后的轨迹数据"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # 保存为npz格式
    npz_filename = f"{options.output_prefix}_aligned_trajectories_{timestamp}.npz"
    npz_path = os.path.join(options.output_dir, npz_filename)
    
    np.savez(npz_path,
             timestamps=timestamps,
             original_estimated=original_est,
             aligned_estimated=aligned_est,
             ground_truth=ground_truth,
             rotation_matrix=R,
             translation_vector=t,
             scale_factor=s,
             alignment_type=options.alignment_type)
    
    print(f"保存对齐轨迹数据: {npz_path}")

def main():
    parser = create_parser()
    (options, args) = parser.parse_args()
    
    # 验证必需参数
    if not options.estimated_poses or not options.ground_truth_poses:
        print("错误: 必须提供 --estimated 和 --ground_truth 参数")
        parser.print_help()
        sys.exit(1)
    
    if options.verbose:
        print("=== ATE 分析开始 ===")
        print(f"估计轨迹: {options.estimated_poses}")
        print(f"真值轨迹: {options.ground_truth_poses}")
        print(f"对齐类型: {options.alignment_type}")
        print(f"输出目录: {options.output_dir}")
    
    try:
        # 1. 加载轨迹数据
        if options.verbose:
            print("\n--- 加载轨迹数据 ---")
        
        timestamps_est, positions_est, has_ts_est = load_trajectory_data(
            options.estimated_poses, options.verbose)
        timestamps_gt, positions_gt, has_ts_gt = load_trajectory_data(
            options.ground_truth_poses, options.verbose)
        
        # 2. 关联轨迹
        if options.verbose:
            print("\n--- 轨迹时间戳关联 ---")
        
        if has_ts_est and has_ts_gt:
            # 都有时间戳，进行关联
            aligned_est, aligned_gt, valid_timestamps = associate_trajectories(
                timestamps_est, positions_est, timestamps_gt, positions_gt, verbose=options.verbose)
        elif not has_ts_est and not has_ts_gt:
            # 都没有时间戳，直接按序号对齐
            min_len = min(len(positions_est), len(positions_gt))
            aligned_est = positions_est[:min_len]
            aligned_gt = positions_gt[:min_len]
            valid_timestamps = np.arange(min_len)
            if options.verbose:
                print(f"  按序号对齐，使用前 {min_len} 个位姿")
        else:
            raise ValueError("估计轨迹与真值轨迹的时间戳格式不一致")
        
        # 限制用于对齐的位姿对数
        if len(aligned_est) > options.max_pairs:
            indices = np.linspace(0, len(aligned_est)-1, options.max_pairs, dtype=int)
            aligned_est_for_align = aligned_est[indices]
            aligned_gt_for_align = aligned_gt[indices]
            if options.verbose:
                print(f"  限制对齐位姿对数为: {options.max_pairs}")
        else:
            aligned_est_for_align = aligned_est
            aligned_gt_for_align = aligned_gt
        
        # 3. 轨迹对齐
        if options.verbose:
            print(f"\n--- 轨迹对齐 ({options.alignment_type.upper()}) ---")
        
        if options.alignment_type.lower() == 'sim3':
            R, t, s = umeyama_alignment(aligned_est_for_align, aligned_gt_for_align, with_scale=True)
            if options.verbose:
                print(f"  尺度因子: {s:.6f}")
        else:
            R, t, s = umeyama_alignment(aligned_est_for_align, aligned_gt_for_align, with_scale=False)
            s = 1.0
        
        if options.verbose:
            print(f"  旋转矩阵行列式: {np.linalg.det(R):.6f}")
            print(f"  平移向量: [{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}]")
        
        # 应用变换到完整轨迹
        aligned_est_transformed = s * (aligned_est @ R.T) + t
        
        # 4. 计算ATE
        if options.verbose:
            print("\n--- 计算ATE指标 ---")
        
        ate_errors = aligned_gt - aligned_est_transformed
        ate_metrics, ate_errors_norm = compute_ate_metrics(ate_errors)
        
        if options.verbose:
            print(f"  RMSE: {ate_metrics['rmse']:.6f} m")
            print(f"  Mean: {ate_metrics['mean']:.6f} m") 
            print(f"  Median: {ate_metrics['median']:.6f} m")
            print(f"  Std: {ate_metrics['std']:.6f} m")
            print(f"  Max: {ate_metrics['max']:.6f} m")
            print(f"  Min: {ate_metrics['min']:.6f} m")
        
        # 5. 创建输出目录
        os.makedirs(options.output_dir, exist_ok=True)
        
        # 6. 绘制图表
        if options.verbose:
            print("\n--- 生成图表 ---")
        
        plot_ate_results(aligned_est, aligned_est_transformed, aligned_gt, 
                        ate_errors_norm, valid_timestamps, ate_metrics, options)
        
        # 7. 保存数值结果
        save_ate_metrics(ate_metrics, ate_errors_norm, valid_timestamps, options)
        
        # 8. 可选保存对齐后轨迹
        if options.save_aligned_traj:
            save_aligned_trajectories(aligned_est, aligned_est_transformed, aligned_gt, 
                                    valid_timestamps, R, t, s, options)
        
        if options.verbose:
            print(f"\n=== ATE 分析完成，结果保存至: {options.output_dir} ===")
        
        print(f"ATE RMSE: {ate_metrics['rmse']:.6f} m")
        
    except Exception as e:
        print(f"错误: {str(e)}")
        if options.verbose:
            import traceback
            traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
