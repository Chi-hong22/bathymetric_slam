#!/usr/bin/env python3
"""
在线 SLAM 误差曲线绘制脚本。
读取 `ping_error.csv`，并根据 `--ping_dt` 将伪 ping 索引转换为时间（秒），
输出 `time vs error_xy` 与 `time vs error_yaw` 曲线，并叠加纯 DR 误差。
"""

import argparse
import os
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(description="Plot online SLAM errors vs time.")
    parser.add_argument("--ping_error_csv",
                        required=True,
                        help="在线日志生成的 ping_error.csv 路径。")
    parser.add_argument("--ping_dt",
                        type=float,
                        default=1.0,
                        help="相邻 ping 的固定时间间隔 (秒)，用于将索引转换为时间。")
    parser.add_argument("--save_fig",
                        default="",
                        help="保存图片的路径或目录（默认留空）。")
    parser.add_argument("--dpi",
                        type=int,
                        default=300,
                        help="保存图片的 DPI，默认 300。")
    parser.add_argument("--no_show",
                        action="store_true",
                        help="仅保存图片而不弹出窗口。")
    return parser.parse_args()


def load_ping_error(csv_path: str) -> np.ndarray:
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"ping_error_csv not found: {csv_path}")
    data = np.genfromtxt(csv_path, delimiter=",", names=True)
    if data.size == 0:
        raise ValueError(f"No data rows in {csv_path}")
    # 当只有一行时，genfromtxt 返回 0 维结构化数组，需要转成 1 维
    if data.shape == ():
        data = np.array([data])
    return data


def resolve_save_path(save_target: str, suffix: str) -> str:
    if not save_target:
        return ""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    default_name = f"online_error_{suffix}_{timestamp}.png"

    target_is_dir = False
    if save_target.endswith(os.sep):
        target_is_dir = True
    else:
        _, ext = os.path.splitext(save_target)
        target_is_dir = (ext == "")

    if target_is_dir:
        directory = save_target if save_target else os.getcwd()
        os.makedirs(directory, exist_ok=True)
        return os.path.join(directory, default_name)

    directory = os.path.dirname(save_target)
    if not directory:
        directory = os.getcwd()
    os.makedirs(directory, exist_ok=True)
    base_name = os.path.splitext(os.path.basename(save_target))[0] or "online_error"
    ext = os.path.splitext(os.path.basename(save_target))[1] or ".png"
    return os.path.join(directory, f"{base_name}_{suffix}_{timestamp}{ext}")


def main():
    args = parse_args()
    if args.dpi <= 0:
        print("Warning: invalid dpi value, fallback to 300.")
        args.dpi = 300

    data = load_ping_error(args.ping_error_csv)
    time_axis = data["ping_index"] * args.ping_dt

    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 8))
    axes[0].plot(time_axis, data["err_xy"], label="Online Estimate", color="#1f77b4", linewidth=1.5)
    axes[0].plot(time_axis, data["err_xy_dr"], label="Pure DR", color="#d62728", linewidth=1.2, alpha=0.8)
    axes[0].set_ylabel("Position Error (m)")
    axes[0].set_title("Position Error vs Time")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")

    axes[1].plot(time_axis, data["err_yaw"], label="Online Estimate", color="#1f77b4", linewidth=1.5)
    axes[1].plot(time_axis, data["err_yaw_dr"], label="Pure DR", color="#d62728", linewidth=1.2, alpha=0.8)
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Yaw Error (rad)")
    axes[1].set_title("Yaw Error vs Time")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best")

    fig.tight_layout()

    save_path = resolve_save_path(args.save_fig, "combined")
    if save_path:
        fig.savefig(save_path, dpi=args.dpi, bbox_inches="tight")
        print(f"[plot_online_error] Saved figure to {save_path}")

    if not args.no_show:
        plt.show()
    else:
        plt.close(fig)


if __name__ == "__main__":
    main()

