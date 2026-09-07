#!/usr/bin/env python3
"""
可视化脚本：从规划结果 JSON 渲染图表

读取 run_planning.py 生成的产物（output/demo/），生成：
- 3D 静态图 (matplotlib, PNG)
- 交互式 3D (plotly, HTML)
- 风险热力图 (matplotlib, PNG)
- 航点指令可视化 (可选)

输出目录：<input>/visualizations/

用法:
    python visualize.py --input ./output/demo
    python visualize.py --input ./output/stress/visualizations
    python visualize.py --input ./output/demo --no-interactive  # 跳过 plotly
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from typing import List, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from src.models.space import SpaceTimeConfig
from src.models.obstacles import (
    ObstacleSet, StaticObstacle, DynamicObstacle,
)
from src.models.task import PlannedRoute, TrajectoryPoint
from src.visualization.viewer import RouteVisualizer


def _parse_obstacles(static_path: str, dynamic_path: str) -> ObstacleSet:
    """从 JSON 文件重新构造 ObstacleSet"""
    obstacle_set = ObstacleSet()

    if static_path and os.path.exists(static_path):
        with open(static_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        for item in data.get("obstacles", []):
            obstacle_set.add_static(StaticObstacle(
                obstacle_id=item.get("id", "unknown"),
                name=item.get("name", "unnamed"),
                obstacle_type=item.get("type", "other"),
                x=float(item["x"]),
                y=float(item["y"]),
                z=float(item.get("z", 0)),
                width=float(item.get("width", 20.0)),
                depth=float(item.get("depth", 20.0)),
                height=float(item.get("height", 50.0)),
                source=item.get("source", "file"),
            ))

    if dynamic_path and os.path.exists(dynamic_path):
        with open(dynamic_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        for item in data.get("obstacles", []):
            trajectory = []
            for p in item.get("trajectory", []):
                trajectory.append((float(p["x"]), float(p["y"]), float(p["z"]), float(p["t"])))
            obstacle_set.add_dynamic(DynamicObstacle(
                obstacle_id=item.get("id", "unknown"),
                name=item.get("name", "unnamed"),
                obstacle_type=item.get("type", "other"),
                trajectory=trajectory,
                safety_radius=float(item.get("safety_radius", 10.0)),
                source=item.get("source", "file"),
            ))

    return obstacle_set


def _load_config(config_path: str) -> SpaceTimeConfig:
    with open(config_path, "r", encoding="utf-8") as f:
        return SpaceTimeConfig(**json.load(f))


def _load_routes(input_dir: str) -> Tuple[List[PlannedRoute], str]:
    """从规划结果 JSON 加载航线列表。优先读取 planning_result_*.json，回退 routes/*.json"""
    # 优先：聚合的 planning_result_*.json
    plan_files = sorted(glob.glob(os.path.join(input_dir, "planning_result_*.json")))
    if plan_files:
        latest = plan_files[-1]
        with open(latest, "r", encoding="utf-8") as f:
            data = json.load(f)
        routes = []
        for rd in data.get("routes", []):
            traj = [
                TrajectoryPoint(
                    x=p["x"], y=p["y"], z=p["z"], t=p["t"],
                    heading=p.get("heading", 0.0),
                )
                for p in rd.get("trajectory", [])
            ]
            metrics = rd.get("metrics", {})
            routes.append(PlannedRoute(
                route_id=rd.get("route_id", "unknown"),
                task_id=rd.get("task_id", "unknown"),
                aircraft_id=rd.get("aircraft_id", "unknown"),
                trajectory=traj,
                status=rd.get("status", "planned"),
                message=rd.get("message", ""),
                algorithm=rd.get("algorithm", ""),
                planning_time_ms=rd.get("planning_time_ms", 0.0),
                total_distance=metrics.get("total_distance_m", 0.0),
                total_time=metrics.get("total_time_s", 0.0),
                total_energy=metrics.get("total_energy", 0.0),
                max_risk=metrics.get("max_risk", 0.0),
                avg_risk=metrics.get("avg_risk", 0.0),
                min_obstacle_distance=metrics.get("min_obstacle_distance_m", 0.0),
            ))
        return routes, latest

    # 回退：routes/*.json
    route_files = sorted(glob.glob(os.path.join(input_dir, "routes", "*.json")))
    if not route_files:
        return [], ""
    routes = []
    for rf in route_files:
        with open(rf, "r", encoding="utf-8") as f:
            rd = json.load(f)
        traj = [
            TrajectoryPoint(
                x=p["x"], y=p["y"], z=p["z"], t=p["t"],
                heading=p.get("heading", 0.0),
            )
            for p in rd.get("trajectory", [])
        ]
        metrics = rd.get("metrics", {})
        routes.append(PlannedRoute(
            route_id=rd.get("route_id", os.path.basename(rf).replace(".json", "")),
            task_id=rd.get("task_id", "unknown"),
            aircraft_id=rd.get("aircraft_id", "unknown"),
            trajectory=traj,
            status=rd.get("status", "planned"),
            message=rd.get("message", ""),
            algorithm=rd.get("algorithm", ""),
            planning_time_ms=rd.get("planning_time_ms", 0.0),
            total_distance=metrics.get("total_distance_m", 0.0),
            total_time=metrics.get("total_time_s", 0.0),
            total_energy=metrics.get("total_energy", 0.0),
            max_risk=metrics.get("max_risk", 0.0),
            avg_risk=metrics.get("avg_risk", 0.0),
            min_obstacle_distance=metrics.get("min_obstacle_distance_m", 0.0),
        ))
    return routes, route_files[-1]


def main():
    parser = argparse.ArgumentParser(description="可视化规划结果")
    parser.add_argument("--input", required=True, help="规划结果目录（含 planning_result_*.json 或 routes/*.json）")
    parser.add_argument("--data-dir", default="./data",
                        help="原始数据目录，含 config.json + static/dynamic_obstacles.json，"
                             "用于恢复障碍物可视化")
    parser.add_argument(
        "--no-interactive", action="store_true",
        help="跳过 Plotly HTML 输出（仅 matplotlib）",
    )
    parser.add_argument(
        "--no-heatmap", action="store_true",
        help="跳过风险热力图",
    )
    args = parser.parse_args()

    input_dir = args.input
    if not os.path.isdir(input_dir):
        print(f"[!] 输入目录不存在: {input_dir}")
        sys.exit(1)

    viz_dir = os.path.join(input_dir, "visualizations")
    os.makedirs(viz_dir, exist_ok=True)

    # 1. 加载规划结果
    print("=" * 60)
    print("  可视化脚本")
    print("=" * 60)
    print(f"\n[1/4] 加载规划结果: {input_dir}")
    routes, source = _load_routes(input_dir)
    if not routes:
        print("[!] 未找到航线数据")
        sys.exit(1)
    print(f"  加载 {len(routes)} 条航线 (来源: {os.path.basename(source)})")
    planned = [r for r in routes if r.status == "planned"]
    print(f"    其中 {len(planned)} 条成功规划")

    # 2. 加载环境（config + obstacles）
    env_snapshot = os.path.join(input_dir, "environment.json")
    if os.path.exists(env_snapshot):
        print(f"\n[2/4] 加载环境快照: {env_snapshot}")
        with open(env_snapshot, "r", encoding="utf-8") as f:
            snap = json.load(f)
        config = SpaceTimeConfig(**snap.get("config", {}))
        obstacle_set = ObstacleSet()

        for item in snap.get("static_obstacles", {}).get("obstacles", []):
            obstacle_set.add_static(StaticObstacle(
                obstacle_id=item.get("id", "unknown"),
                name=item.get("name", "unnamed"),
                obstacle_type=item.get("type", "other"),
                x=float(item["x"]),
                y=float(item["y"]),
                z=float(item.get("z", 0)),
                width=float(item.get("width", 20.0)),
                depth=float(item.get("depth", 20.0)),
                height=float(item.get("height", 50.0)),
                source=item.get("source", "file"),
            ))

        for item in snap.get("dynamic_obstacles", {}).get("obstacles", []):
            trajectory = [
                (float(p["x"]), float(p["y"]), float(p["z"]), float(p["t"]))
                for p in item.get("trajectory", [])
            ]
            obstacle_set.add_dynamic(DynamicObstacle(
                obstacle_id=item.get("id", "unknown"),
                name=item.get("name", "unnamed"),
                obstacle_type=item.get("type", "other"),
                trajectory=trajectory,
                safety_radius=float(item.get("safety_radius", 10.0)),
                source=item.get("source", "file"),
            ))
    else:
        print(f"\n[2/4] 加载环境配置: {args.data_dir}")
        config_path = os.path.join(args.data_dir, "config.json")
        static_path = os.path.join(args.data_dir, "static_obstacles.json")
        dynamic_path = os.path.join(args.data_dir, "dynamic_obstacles.json")

        if not os.path.exists(config_path):
            print(f"[!] 缺少 config.json: {config_path}")
            sys.exit(1)
        config = _load_config(config_path)
        obstacle_set = _parse_obstacles(static_path, dynamic_path)

    print(f"  时空网格: {config.nx}x{config.ny}x{config.nz}x{config.nt}")
    print(f"  障碍物: {len(obstacle_set.static_obstacles)} 静态 + "
          f"{len(obstacle_set.dynamic_obstacles)} 动态")

    if not planned:
        print("[!] 没有成功规划的航线，跳过可视化")
        sys.exit(0)

    # 3. 生成可视化
    print(f"\n[3/4] 生成可视化 → {viz_dir}/")
    visualizer = RouteVisualizer(config, obstacle_set)

    # 3.1 Matplotlib 3D 静态图
    out_png = os.path.join(viz_dir, "routes_3d.png")
    visualizer.plot_matplotlib_3d(
        planned, out_png,
        title=f"低空无人机路径规划 3D ({len(planned)} 条航线)",
    )
    print(f"  ✓ 3D 静态图: {out_png}")

    # 3.2 Plotly 交互式 HTML
    if not args.no_interactive:
        out_html = os.path.join(viz_dir, "routes_interactive.html")
        visualizer.generate_plotly_html(
            planned, out_html,
            title=f"低空无人机路径规划交互式 ({len(planned)} 条航线)",
        )
        print(f"  ✓ 3D 交互式 HTML: {out_html}")

    # 3.3 风险热力图（第一条航线）
    if not args.no_heatmap:
        out_hm = os.path.join(viz_dir, "risk_heatmap.png")
        visualizer.plot_risk_heatmap(planned[0], out_hm)
        print(f"  ✓ 风险热力图: {out_hm}")

    # 3.4 多航线对比柱状图（如果有 metrics）
    out_bar = os.path.join(viz_dir, "route_metrics_bar.png")
    _plot_route_metrics(planned, out_bar)
    if os.path.exists(out_bar):
        print(f"  ✓ 航线指标柱状图: {out_bar}")

    # 4. 完成
    print(f"\n[4/4] 完成 ✓")
    print(f"  共生成 {len(os.listdir(viz_dir))} 个产物")
    print(f"  输出目录: {os.path.abspath(viz_dir)}")
    print("=" * 60)


def _plot_route_metrics(routes: List[PlannedRoute], output_path: str):
    """为成功规划的航线绘制 4 个指标的柱状图"""
    if not routes:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    labels = [f"{r.route_id[:18]}\n({r.task_id})" for r in routes]
    x = np.arange(len(routes))

    metrics = {
        "航程 (m)": [r.total_distance for r in routes],
        "时间 (s)": [r.total_time for r in routes],
        "最大风险": [r.max_risk for r in routes],
        "平均风险": [r.avg_risk for r in routes],
    }

    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    axes = axes.flatten()
    colors = ["#1f77b4", "#ff7f0e", "#d62728", "#9467bd"]

    for ax, (name, values), color in zip(axes, metrics.items(), colors):
        ax.bar(x, values, color=color, alpha=0.75)
        ax.set_title(name, fontsize=12)
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=8)
        ax.grid(axis="y", alpha=0.3)

    fig.suptitle("各航线指标对比", fontsize=14, fontweight="bold")
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()


if __name__ == "__main__":
    main()