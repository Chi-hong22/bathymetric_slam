#!/usr/bin/python

import matplotlib.pyplot as plot
import numpy
import sys
import os
from datetime import datetime
from optparse import OptionParser

parser = OptionParser()
parser.add_option("--initial_poses",
                  dest="initial_poses",
                  default="",
                  help="The filename that contains the original poses.")
parser.add_option("--corrupted_poses",
                  dest="corrupted_poses",
                  default="",
                  help="The filename that contains the optimized poses.")
parser.add_option("--optimized_poses",
                  dest="optimized_poses",
                  default="",
                  help="The filename that contains the optimized poses.")
parser.add_option("-e",
                  "--axes_equal",
                  action="store_true",
                  dest="axes_equal",
                  default="",
                  help="Make the plot axes equal.")
parser.add_option("--output_file",
                  dest="outputFile",
                  default="",
                  help="The output file.")
parser.add_option("--save_fig",
                  dest="save_fig",
                  default="build/",
                  help="Save figure to specified path or directory.")
parser.add_option("--no_show",
                  action="store_true",
                  dest="no_show",
                  default=False,
                  help="Do not display the plot window.")
parser.add_option("--dpi",
                  dest="dpi",
                  type="int",
                  default=300,
                  help="Figure resolution for saving (default: 300).")
parser.add_option("--view",
                  dest="view",
                  default="top",
                  help="View angle: 'top' for overhead/2D view, '3d' for 3D view (default: top).")

(options, args) = parser.parse_args()

# 读取原始和优化后的位姿文件
poses_original = None
if options.initial_poses != '':
    poses_original = numpy.genfromtxt(options.initial_poses, usecols=(1, 2, 3))

poses_corrupted = None
if options.corrupted_poses != '':
    poses_corrupted = numpy.genfromtxt(options.corrupted_poses,
                                       usecols=(1, 2, 3))

poses_optimized = None
if options.optimized_poses != '':
    poses_optimized = numpy.genfromtxt(options.optimized_poses,
                                       usecols=(1, 2, 3))

# 检查是否有任何位姿数据被成功加载
if poses_original is None and poses_corrupted is None and poses_optimized is None:
    print("No poses loaded. Exit.")
    sys.exit(0)

# 验证 DPI 参数
if options.dpi <= 0:
    print("Warning: Invalid DPI value, using default 300")
    options.dpi = 300

# 计算位姿差异的平均值 (仅在有原始位姿时计算)
if poses_original is not None and poses_corrupted is not None:
    n_corr = min(len(poses_original), len(poses_corrupted))
    if len(poses_original) != len(poses_corrupted):
        print(f"Warning: Length mismatch between original ({len(poses_original)}) and corrupted ({len(poses_corrupted)}), using first {n_corr} poses")
    
    if n_corr > 0:
        sum_corr = 0.0
        for i in range(n_corr):
            sum_corr += numpy.linalg.norm(poses_original[i, :] - poses_corrupted[i, :])
        sum_corr /= n_corr
        print("Diff original and corrupted", sum_corr)
    else:
        print("Warning: zero-length pose array in diff computation, skipped.")

if poses_original is not None and poses_optimized is not None:
    n_opt = min(len(poses_original), len(poses_optimized))
    if len(poses_original) != len(poses_optimized):
        print(f"Warning: Length mismatch between original ({len(poses_original)}) and optimized ({len(poses_optimized)}), using first {n_opt} poses")
    
    if n_opt > 0:
        sum_opt = 0.0
        for i in range(n_opt):
            sum_opt += numpy.linalg.norm(poses_original[i, :] - poses_optimized[i, :])
        sum_opt /= n_opt
        print("Diff original and optimized", sum_opt)
    else:
        print("Warning: zero-length pose array in diff computation, skipped.")

#  with open(options.outputFile, "a") as text_file:
#  text_file.write("%s" % sum_corr)
#  text_file.write(" %s" % sum_opt)
#  text_file.close()
#
# 绘制指定的位姿结果
figure = plot.figure()

# 根据视图参数选择绘图模式
if options.view.lower() == '3d':
    axes = figure.add_subplot(111, projection='3d')
    plot_3d = True
else:
    axes = figure.add_subplot(111)
    plot_3d = False

if poses_original is not None:
    if plot_3d:
        plot.plot(poses_original[:, 0],
                  poses_original[:, 1],
                  poses_original[:, 2],
                  '-',
                  alpha=0.8,
                  color="green",
                  linewidth=1.5,
                  label='Ground Truth')
    else:
        plot.plot(poses_original[:, 0],
                  poses_original[:, 1],
                  '-',
                  alpha=0.8,
                  color="green",
                  linewidth=1.5,
                  label='Ground Truth')

if poses_corrupted is not None:
    if plot_3d:
        plot.plot(poses_corrupted[:, 0],
                  poses_corrupted[:, 1],
                  poses_corrupted[:, 2],
                  '-',
                  alpha=0.7,
                  color="red",
                  linewidth=1.5,
                  label='Corrupted')
    else:
        plot.plot(poses_corrupted[:, 0],
                  poses_corrupted[:, 1],
                  '-',
                  alpha=0.7,
                  color="red",
                  linewidth=1.5,
                  label='Corrupted')

if poses_optimized is not None:
    if plot_3d:
        plot.plot(poses_optimized[:, 0],
                  poses_optimized[:, 1],
                  poses_optimized[:, 2],
                  '-',
                  alpha=0.8,
                  color="blue",
                  linewidth=1.5,
                  label='Optimized')
    else:
        plot.plot(poses_optimized[:, 0],
                  poses_optimized[:, 1],
                  '-',
                  alpha=0.8,
                  color="blue",
                  linewidth=1.5,
                  label='Optimized')

# 根据数据范围设置纵横比，以实现真实比例表示
all_poses = []
if poses_original is not None:
    all_poses.append(poses_original)
if poses_corrupted is not None:
    all_poses.append(poses_corrupted)
if poses_optimized is not None:
    all_poses.append(poses_optimized)

if all_poses:
    combined_poses = numpy.vstack(all_poses)
    min_coords = numpy.min(combined_poses, axis=0)
    max_coords = numpy.max(combined_poses, axis=0)
    ranges = max_coords - min_coords
    # 为避免除以零或极小范围，设置最小范围
    ranges[ranges < 1e-6] = 1e-6
    
    if plot_3d:
        axes.set_box_aspect(ranges)
    else:
        # 2D视图设置等比例
        axes.set_aspect('equal', adjustable='box')

plot.legend()
if plot_3d:
    plot.title('Trajectories Comparison (3D View)')
    plot.xlabel('X')
    plot.ylabel('Y')
    axes.set_zlabel('Z')
else:
    plot.title('Trajectories Comparison (Top View)')
    plot.xlabel('X (m)')
    plot.ylabel('Y (m)')
    plot.grid(True, alpha=0.3)
    # 将图例放在右上角外侧，避免与轨迹重合
    plot.legend(bbox_to_anchor=(1.05, 1), loc='upper left')

# 处理图片保存
if options.save_fig:
    save_path = options.save_fig
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    view_suffix = "3d" if options.view.lower() == '3d' else "top"
    
    # 判断是否为目录（无扩展名或以/结尾）
    if os.path.isdir(save_path) or (not os.path.splitext(save_path)[1] and not save_path.endswith('.')):
        # 视为目录，生成自动文件名
        if not os.path.exists(save_path):
            os.makedirs(save_path, exist_ok=True)
        filename = f"plot_results_{view_suffix}_{timestamp}.png"
        full_path = os.path.join(save_path, filename)
    else:
        # 视为文件路径，在文件名中插入时间戳和视角
        dir_part = os.path.dirname(save_path)
        base_name = os.path.basename(save_path)
        name_part, ext_part = os.path.splitext(base_name)
        
        # 构造新文件名：原名_视角_时间戳.扩展名
        new_filename = f"{name_part}_{view_suffix}_{timestamp}{ext_part if ext_part else '.png'}"
        
        if dir_part and not os.path.exists(dir_part):
            # 如果没有目录部分，使用轨迹文件所在目录
            if not dir_part:
                if options.initial_poses:
                    dir_part = os.path.dirname(os.path.abspath(options.initial_poses))
                elif options.corrupted_poses:
                    dir_part = os.path.dirname(os.path.abspath(options.corrupted_poses))
                elif options.optimized_poses:
                    dir_part = os.path.dirname(os.path.abspath(options.optimized_poses))
                else:
                    dir_part = os.getcwd()
                full_path = os.path.join(dir_part, new_filename)
            else:
                os.makedirs(dir_part, exist_ok=True)
                full_path = os.path.join(dir_part, new_filename)
        else:
            if dir_part:
                full_path = os.path.join(dir_part, new_filename)
            else:
                # 使用当前目录或轨迹文件目录
                if options.initial_poses:
                    dir_part = os.path.dirname(os.path.abspath(options.initial_poses))
                elif options.corrupted_poses:
                    dir_part = os.path.dirname(os.path.abspath(options.corrupted_poses))
                elif options.optimized_poses:
                    dir_part = os.path.dirname(os.path.abspath(options.optimized_poses))
                else:
                    dir_part = os.getcwd()
                full_path = os.path.join(dir_part, new_filename)
    
    plot.savefig(full_path, dpi=options.dpi, bbox_inches='tight')
    print(f"Saved figure to {full_path}")

# 处理显示控制
if not options.no_show:
    plot.show()
else:
    print("Figure display skipped (--no_show).")
