#!/usr/bin/env python3
"""
压力测试可视化脚本

从 run_stress_test.py 生成的 stress_results_*.json 渲染对比图表：
- 墙钟时间 vs 规模 N (FULL vs DISPATCH)
- 加速比 vs 规模 N
- 成功率 vs 规模 N
- 吞吐 vs 规模 N
- 单任务耗时分布 (P50/P95/P99/max)

输出目录：<input>/visualizations/

用法:
    python visualize_stress.py --input ./output/stress
    python visualize_stress.py --input ./output/stress_compare
    python visualize_stress.py --input ./output/stress --no-bar
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from typing import Dict, List, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def _load_results(input_dir: str) -> Dict[str, List[dict]]:
    """加载压力测试结果。优先匹配 *_full.json / *_dispatch.json，回退 stress_results.json"""
    results: Dict[str, List[dict]] = {}

    # 1) 比较模式：分别有 *_full.json 和 *_dispatch.json
    # 注意：用 *full.json 而非 *_full.json，避免要求中间必须有字符
    full_files = sorted(glob.glob(os.path.join(input_dir, "stress_results*full.json")))
    disp_files = sorted(glob.glob(os.path.join(input_dir, "stress_results*dispatch.json")))
    if full_files:
        with open(full_files[-1], "r", encoding="utf-8") as f:
            results["FULL"] = json.load(f)
    if disp_files:
        with open(disp_files[-1], "r", encoding="utf-8") as f:
            results["DISPATCH"] = json.load(f)

    # 2) 单模式：仅 stress_results.json
    if not results:
        single = os.path.join(input_dir, "stress_results.json")
        if os.path.exists(single):
            with open(single, "r", encoding="utf-8") as f:
                results["FULL"] = json.load(f)

    # 3) 兼容旧版本 stress_results_interim_*.json
    if not results:
        for label, pattern in [("FULL", "stress_results_interim_full.json"),
                               ("DISPATCH", "stress_results_interim_dispatch.json")]:
            p = os.path.join(input_dir, pattern)
            if os.path.exists(p):
                with open(p, "r", encoding="utf-8") as f:
                    results[label] = json.load(f)

    return results


def main():
    parser = argparse.ArgumentParser(description="压力测试结果可视化")
    parser.add_argument("--input", required=True, help="压力测试输出目录")
    parser.add_argument(
        "--no-bar", action="store_true",
        help="跳过柱状图（墙钟/吞吐）",
    )
    parser.add_argument(
        "--no-percentile", action="store_true",
        help="跳过单任务耗时分布图",
    )
    args = parser.parse_args()

    input_dir = args.input
    if not os.path.isdir(input_dir):
        print(f"[!] 输入目录不存在: {input_dir}")
        sys.exit(1)

    viz_dir = os.path.join(input_dir, "visualizations")
    os.makedirs(viz_dir, exist_ok=True)

    print("=" * 60)
    print("  压力测试结果可视化")
    print("=" * 60)
    print(f"\n[1/3] 加载压力测试结果: {input_dir}")

    results = _load_results(input_dir)
    if not results:
        print("[!] 未找到 stress_results_*.json")
        sys.exit(1)
    for mode, data in results.items():
        print(f"  [{mode}] {len(data)} 个规模: {[r['scale'] for r in data]}")

    is_compare = "FULL" in results and "DISPATCH" in results

    print(f"\n[2/3] 生成可视化 → {viz_dir}/")

    if not args.no_bar:
        _plot_wall_clock(results, os.path.join(viz_dir, "wall_clock.png"))
        print(f"  ✓ 墙钟时间对比: {viz_dir}/wall_clock.png")

        if is_compare:
            _plot_speedup(results, os.path.join(viz_dir, "speedup.png"))
            print(f"  ✓ 加速比对比:   {viz_dir}/speedup.png")

        _plot_success_rate(results, os.path.join(viz_dir, "success_rate.png"))
        print(f"  ✓ 成功率对比:   {viz_dir}/success_rate.png")

        _plot_throughput(results, os.path.join(viz_dir, "throughput.png"))
        print(f"  ✓ 吞吐对比:     {viz_dir}/throughput.png")

        if is_compare:
            _plot_full_vs_dispatch(results, os.path.join(viz_dir, "full_vs_dispatch.png"))
            print(f"  ✓ FULL vs DISPATCH 综合: {viz_dir}/full_vs_dispatch.png")

    if not args.no_percentile:
        _plot_percentile(results, os.path.join(viz_dir, "task_time_percentiles.png"))
        print(f"  ✓ 单任务耗时分布: {viz_dir}/task_time_percentiles.png")

    print(f"\n[3/3] 完成 ✓")
    print(f"  共生成 {len(os.listdir(viz_dir))} 个产物")
    print(f"  输出目录: {os.path.abspath(viz_dir)}")
    print("=" * 60)


# ============================================================
# 绘图函数
# ============================================================

def _plot_wall_clock(results: Dict[str, List[dict]], output_path: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    fig, ax = plt.subplots(figsize=(10, 6))
    colors = {"FULL": "#1f77b4", "DISPATCH": "#2ca02c"}
    for mode, data in results.items():
        scales = [r["scale"] for r in data]
        wall = [r["wall_clock_time_s"] for r in data]
        ax.plot(scales, wall, "o-", color=colors.get(mode, "#888"),
                linewidth=2, markersize=8, label=mode)
        for x, y in zip(scales, wall):
            ax.annotate(f"{y:.1f}s", (x, y), textcoords="offset points",
                        xytext=(8, 0), fontsize=8,
                        color=colors.get(mode, "#888"))
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("规模 N (任务数)", fontsize=12)
    ax.set_ylabel("墙钟时间 (s)", fontsize=12)
    ax.set_title("墙钟时间 vs 任务规模", fontsize=14, fontweight="bold")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(fontsize=10)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()


def _plot_speedup(results: Dict[str, List[dict]], output_path: str):
    if "FULL" not in results or "DISPATCH" not in results:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    full_by_n = {r["scale"]: r["wall_clock_time_s"] for r in results["FULL"]}
    disp_by_n = {r["scale"]: r["wall_clock_time_s"] for r in results["DISPATCH"]}
    scales = sorted(set(full_by_n.keys()) & set(disp_by_n.keys()))
    speedups = [full_by_n[n] / disp_by_n[n] if disp_by_n[n] > 0 else 0 for n in scales]

    fig, ax = plt.subplots(figsize=(10, 6))
    bars = ax.bar(scales, speedups, color="#2ca02c", alpha=0.75, edgecolor="black")
    for bar, s in zip(bars, speedups):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.01,
                f"{s:.2f}×", ha="center", va="bottom", fontsize=11, fontweight="bold")
    ax.set_xlabel("规模 N", fontsize=12)
    ax.set_ylabel("DISPATCH 加速比 (FULL/DISPATCH)", fontsize=12)
    ax.set_title("DISPATCH 模式相对 FULL 模式的加速比", fontsize=14, fontweight="bold")
    ax.grid(axis="y", alpha=0.3)
    ax.set_xticks(scales)
    ax.set_xticklabels([str(s) for s in scales])
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()


def _plot_success_rate(results: Dict[str, List[dict]], output_path: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    fig, ax = plt.subplots(figsize=(10, 6))
    colors = {"FULL": "#1f77b4", "DISPATCH": "#2ca02c"}

    # 对齐不同模式的规模列表（支持某个模式中断导致规模缺失）
    all_scales = sorted({r["scale"] for data in results.values() for r in data})
    x = np.arange(len(all_scales))
    width = 0.8 / max(1, len(results))

    for i, (mode, data) in enumerate(results.items()):
        by_n = {r["scale"]: r["success_rate"] for r in data}
        rates = [by_n.get(n, float("nan")) for n in all_scales]
        offset = (i - (len(results) - 1) / 2) * width
        bars = ax.bar(x + offset, rates, width, color=colors.get(mode, "#888"),
                      alpha=0.75, label=mode, edgecolor="black")
        for bar, r in zip(bars, rates):
            if r != r:  # NaN
                continue
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5,
                    f"{r:.1f}%", ha="center", va="bottom", fontsize=8)

    ax.set_xlabel("规模 N", fontsize=12)
    ax.set_ylabel("成功率 (%)", fontsize=12)
    ax.set_title("规划成功率", fontsize=14, fontweight="bold")
    ax.set_xticks(x)
    ax.set_xticklabels([str(s) for s in all_scales])
    ax.set_ylim(80, 102)
    ax.grid(axis="y", alpha=0.3)
    ax.legend(fontsize=10)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()


def _plot_throughput(results: Dict[str, List[dict]], output_path: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 6))
    colors = {"FULL": "#1f77b4", "DISPATCH": "#2ca02c"}
    for mode, data in results.items():
        scales = [r["scale"] for r in data]
        th = [r["throughput_tasks_per_s"] for r in data]
        ax.plot(scales, th, "s-", color=colors.get(mode, "#888"),
                linewidth=2, markersize=8, label=mode)
    ax.set_xscale("log")
    ax.set_xlabel("规模 N (任务数)", fontsize=12)
    ax.set_ylabel("吞吐 (tasks/s)", fontsize=12)
    ax.set_title("吞吐 vs 任务规模", fontsize=14, fontweight="bold")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(fontsize=10)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()


def _plot_full_vs_dispatch(results: Dict[str, List[dict]], output_path: str):
    """综合对比图：3 行 × 2 列小图"""
    if "FULL" not in results or "DISPATCH" not in results:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    full_by_n = {r["scale"]: r for r in results["FULL"]}
    disp_by_n = {r["scale"]: r for r in results["DISPATCH"]}
    scales = sorted(set(full_by_n.keys()) & set(disp_by_n.keys()))

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # (0,0): wall clock 对比
    ax = axes[0, 0]
    fr = [full_by_n[n]["wall_clock_time_s"] for n in scales]
    dr = [disp_by_n[n]["wall_clock_time_s"] for n in scales]
    x = np.arange(len(scales))
    ax.bar(x - 0.2, fr, 0.4, label="FULL", color="#1f77b4", alpha=0.75)
    ax.bar(x + 0.2, dr, 0.4, label="DISPATCH", color="#2ca02c", alpha=0.75)
    ax.set_yscale("log")
    ax.set_xticks(x)
    ax.set_xticklabels([str(s) for s in scales])
    ax.set_xlabel("规模 N")
    ax.set_ylabel("墙钟 (s, log)")
    ax.set_title("墙钟时间")
    ax.legend()
    ax.grid(True, which="both", axis="y", alpha=0.3)

    # (0,1): 加速比
    ax = axes[0, 1]
    speedups = [f / d if d > 0 else 0 for f, d in zip(fr, dr)]
    bars = ax.bar(scales, speedups, color="#2ca02c", alpha=0.75, edgecolor="black")
    for bar, s in zip(bars, speedups):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.01,
                f"{s:.2f}×", ha="center", fontsize=9, fontweight="bold")
    ax.set_xticks(scales)
    ax.set_xticklabels([str(s) for s in scales])
    ax.set_xlabel("规模 N")
    ax.set_ylabel("加速比")
    ax.set_title("DISPATCH vs FULL 加速比")
    ax.grid(axis="y", alpha=0.3)

    # (1,0): 成功率
    ax = axes[1, 0]
    fr_succ = [full_by_n[n]["success_rate"] for n in scales]
    dr_succ = [disp_by_n[n]["success_rate"] for n in scales]
    ax.bar(x - 0.2, fr_succ, 0.4, label="FULL", color="#1f77b4", alpha=0.75)
    ax.bar(x + 0.2, dr_succ, 0.4, label="DISPATCH", color="#2ca02c", alpha=0.75)
    ax.set_xticks(x)
    ax.set_xticklabels([str(s) for s in scales])
    ax.set_xlabel("规模 N")
    ax.set_ylabel("成功率 (%)")
    ax.set_title("规划成功率")
    ax.set_ylim(80, 102)
    ax.legend()
    ax.grid(axis="y", alpha=0.3)

    # (1,1): 吞吐
    ax = axes[1, 1]
    fr_th = [full_by_n[n]["throughput_tasks_per_s"] for n in scales]
    dr_th = [disp_by_n[n]["throughput_tasks_per_s"] for n in scales]
    ax.bar(x - 0.2, fr_th, 0.4, label="FULL", color="#1f77b4", alpha=0.75)
    ax.bar(x + 0.2, dr_th, 0.4, label="DISPATCH", color="#2ca02c", alpha=0.75)
    ax.set_xticks(x)
    ax.set_xticklabels([str(s) for s in scales])
    ax.set_xlabel("规模 N")
    ax.set_ylabel("吞吐 (tasks/s)")
    ax.set_title("吞吐对比")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)

    fig.suptitle("FULL vs DISPATCH 综合对比", fontsize=16, fontweight="bold")
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()


def _plot_percentile(results: Dict[str, List[dict]], output_path: str):
    """单任务耗时分布（P50 / P95 / P99 / max）"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    metrics = [
        ("p50_task_time_ms", "P50"),
        ("p95_task_time_ms", "P95"),
        ("p99_task_time_ms", "P99"),
        ("max_task_time_ms", "Max"),
    ]
    fig, axes = plt.subplots(1, 4, figsize=(18, 5), sharey=True)
    colors = {"FULL": "#1f77b4", "DISPATCH": "#2ca02c"}

    for ax, (field, label) in zip(axes, metrics):
        for mode, data in results.items():
            scales = [r["scale"] for r in data]
            values = [r[field] for r in data]
            ax.plot(scales, values, "o-", color=colors.get(mode, "#888"),
                    linewidth=2, markersize=7, label=mode)
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.set_xlabel("规模 N")
        ax.set_title(label, fontsize=12, fontweight="bold")
        ax.grid(True, which="both", alpha=0.3)
        if label == "P50":
            ax.set_ylabel("单任务耗时 (ms)")
        if label == "Max":
            ax.legend(fontsize=9, loc="upper left")

    fig.suptitle("单任务规划耗时分布 (FULL vs DISPATCH)", fontsize=14, fontweight="bold")
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()


if __name__ == "__main__":
    main()