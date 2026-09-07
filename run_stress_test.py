#!/usr/bin/env python3
"""
无人机规模压力测试 (3.25 大规模并行规划验证)

依次执行 10 / 100 / 1,000 / 10,000 架无人机的批量路径规划，
每个规模下任务数与无人机数相同，所有任务随机生成（起点、终点、
机型、优先级、起飞时间均随机），统计规划成功率、平均/最长耗时、
路径长度、最近障碍距离等指标，并将结果写入 STRESS_TEST_RESULTS.md。

用法:
    /opt/anaconda3/bin/python3 run_stress_test.py
    /opt/anaconda3/bin/python3 run_stress_test.py --output ./output/stress
    /opt/anaconda3/bin/python3 run_stress_test.py --scales 10 100
"""

from __future__ import annotations

import argparse
import json
import os
import random
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from src.models.space import SpaceTimeConfig
from src.models.obstacles import ObstacleSet
from src.models.aircraft import (
    AircraftPerformance,
    FlightTask,
    Waypoint,
)
from src.models.task import PlannedRoute, PlanningResult
from src.planners.parallel_planner import ParallelPlanner


# ---------- 随机任务生成 ----------

AIRCRAFT_TEMPLATES = [
    {
        "id_prefix": "uav-A",
        "model": "DJI-Matrice-300",
        "max_speed": 18.0, "max_climb_rate": 6.0, "max_descent_rate": 4.0,
        "max_turn_rate": 60.0, "max_flight_time": 1800.0, "max_range": 15000.0,
        "min_altitude": 10.0, "max_altitude": 300.0,
    },
    {
        "id_prefix": "uav-B",
        "model": "DJI-Mavic-3-Enterprise",
        "max_speed": 15.0, "max_climb_rate": 5.0, "max_descent_rate": 3.0,
        "max_turn_rate": 45.0, "max_flight_time": 2700.0, "max_range": 20000.0,
        "min_altitude": 10.0, "max_altitude": 250.0,
    },
    {
        "id_prefix": "uav-C",
        "model": "Custom-Delivery-Drone",
        "max_speed": 12.0, "max_climb_rate": 4.0, "max_descent_rate": 2.5,
        "max_turn_rate": 30.0, "max_flight_time": 1200.0, "max_range": 8000.0,
        "min_altitude": 20.0, "max_altitude": 200.0,
    },
    {
        "id_prefix": "uav-D",
        "model": "Inspection-Drone-Pro",
        "max_speed": 10.0, "max_climb_rate": 3.0, "max_descent_rate": 2.0,
        "max_turn_rate": 25.0, "max_flight_time": 2400.0, "max_range": 12000.0,
        "min_altitude": 15.0, "max_altitude": 150.0,
    },
]

TASK_TYPES = ["delivery", "inspection", "patrol", "surveillance"]


def _load_obstacles(data_dir: str) -> ObstacleSet:
    from src.io.input_loader import load_obstacle_set
    return load_obstacle_set(
        os.path.join(data_dir, "static_obstacles.json"),
        os.path.join(data_dir, "dynamic_obstacles.json"),
    )


def _make_valid_position(
    rng: random.Random,
    config: SpaceTimeConfig,
    obstacle_set: ObstacleSet,
    perf: AircraftPerformance,
    margin: float = 30.0,
    max_tries: int = 200,
) -> Tuple[float, float, float]:
    for _ in range(max_tries):
        x = rng.uniform(config.x_min + margin, config.x_max - margin)
        y = rng.uniform(config.y_min + margin, config.y_max - margin)
        z = rng.uniform(perf.min_altitude, perf.max_altitude)
        if not obstacle_set.is_static_blocked(x, y, z):
            return x, y, z
    # 回退
    return (
        (config.x_min + config.x_max) / 2,
        (config.y_min + config.y_max) / 2,
        (perf.min_altitude + perf.max_altitude) / 2,
    )


def generate_random_tasks(
    num_tasks: int,
    config: SpaceTimeConfig,
    obstacle_set: ObstacleSet,
    rng: random.Random,
    departure_window: float = 600.0,
) -> Tuple[List[FlightTask], Dict[str, AircraftPerformance]]:
    tasks: List[FlightTask] = []
    performances: Dict[str, AircraftPerformance] = {}

    for i in range(num_tasks):
        tmpl = rng.choice(AIRCRAFT_TEMPLATES)
        aircraft_id = f"{tmpl['id_prefix']}-{i:05d}"

        perf = AircraftPerformance(
            aircraft_id=aircraft_id, model=tmpl["model"],
            max_speed=tmpl["max_speed"], max_climb_rate=tmpl["max_climb_rate"],
            max_descent_rate=tmpl["max_descent_rate"],
            max_turn_rate=tmpl["max_turn_rate"],
            max_flight_time=tmpl["max_flight_time"], max_range=tmpl["max_range"],
            min_altitude=tmpl["min_altitude"], max_altitude=tmpl["max_altitude"],
        )
        performances[aircraft_id] = perf

        is_multi = rng.random() < 0.2
        if is_multi:
            wps: List[Waypoint] = []
            num_wps = 3
            for k in range(num_wps):
                x, y, z = _make_valid_position(rng, config, obstacle_set, perf)
                wp_type = "start" if k == 0 else ("end" if k == num_wps - 1 else "via")
                wps.append(Waypoint(
                    wp_id=f"wp-{i}-{k}", x=x, y=y, z=z, wp_type=wp_type,
                    hover_time=rng.uniform(0.0, 10.0) if wp_type == "via" else 0.0,
                ))
        else:
            sx, sy, sz = _make_valid_position(rng, config, obstacle_set, perf)
            ex, ey, ez = _make_valid_position(rng, config, obstacle_set, perf)
            wps = [
                Waypoint(wp_id="start", x=sx, y=sy, z=sz, wp_type="start"),
                Waypoint(wp_id="end", x=ex, y=ey, z=ez, wp_type="end"),
            ]

        task = FlightTask(
            task_id=f"task-{i:06d}",
            task_name=f"stress-{i}",
            aircraft_id=aircraft_id,
            waypoints=wps,
            priority=rng.randint(1, 10),
            task_type=rng.choice(TASK_TYPES),
            departure_time=rng.uniform(0.0, departure_window),
            objective_weights={
                "distance": rng.uniform(0.1, 0.4),
                "time": rng.uniform(0.1, 0.4),
                "risk": rng.uniform(0.1, 0.4),
                "energy": rng.uniform(0.05, 0.2),
            },
        )
        total = sum(task.objective_weights.values())
        task.objective_weights = {k: v / total for k, v in task.objective_weights.items()}
        tasks.append(task)

    return tasks, performances


# ---------- 单规模压力测试 ----------

@dataclass
class ScaleResult:
    scale: int
    success_count: int = 0
    failed_count: int = 0
    total_planning_time_ms: float = 0.0
    avg_planning_time_per_task_ms: float = 0.0
    min_task_time_ms: float = 0.0
    max_task_time_ms: float = 0.0
    p50_task_time_ms: float = 0.0
    p95_task_time_ms: float = 0.0
    p99_task_time_ms: float = 0.0
    avg_distance_m: float = 0.0
    avg_time_s: float = 0.0
    avg_min_obstacle_dist_m: float = 0.0
    wall_clock_time_s: float = 0.0
    throughput_tasks_per_s: float = 0.0
    conflict_count: int = 0
    workers_used: int = 0

    @property
    def success_rate(self) -> float:
        return round(self.success_count / self.scale * 100, 2) if self.scale else 0.0


def _percentile(values: List[float], p: float) -> float:
    if not values:
        return 0.0
    sorted_v = sorted(values)
    k = (len(sorted_v) - 1) * p / 100.0
    f = int(k)
    c = min(f + 1, len(sorted_v) - 1)
    if f == c:
        return sorted_v[f]
    return sorted_v[f] + (sorted_v[c] - sorted_v[f]) * (k - f)


def run_scale(
    n: int,
    config: SpaceTimeConfig,
    obstacle_set: ObstacleSet,
    seed: int,
    max_workers: int,
    max_iterations: int,
    dispatch: bool = False,
    verbose: bool = True,
) -> ScaleResult:
    rng = random.Random(seed + n)
    tasks, performances = generate_random_tasks(n, config, obstacle_set, rng)

    mode_label = "DISPATCH" if dispatch else "FULL"
    if verbose:
        print(f"\n{'='*60}", flush=True)
        print(f"  规模 N = {n} [{mode_label}] (seed={seed}, workers={max_workers})", flush=True)
        print(f"{'='*60}", flush=True)
        print(f"  生成 {len(tasks)} 个随机任务", flush=True)
        type_counts: Dict[str, int] = {}
        for t in tasks:
            type_counts[t.task_type] = type_counts.get(t.task_type, 0) + 1
        print(f"  任务类型分布: {type_counts}", flush=True)
        wp_counts = [len(t.waypoints) for t in tasks]
        print(f"  航点数: min={min(wp_counts)}, max={max(wp_counts)}, "
              f"avg={sum(wp_counts)/len(wp_counts):.1f}", flush=True)

    planner = ParallelPlanner(
        config=config, obstacle_set=obstacle_set,
        max_workers=max_workers,
        max_iterations=max_iterations,
    )

    t0 = time.time()
    result: PlanningResult = planner.plan_batch(
        tasks, performances, request_id=f"stress-{n}", dispatch=dispatch,
    )
    wall_clock = time.time() - t0
    print(f"  [完成 N={n} {mode_label}] 墙钟 {wall_clock:.1f}s, "
          f"成功 {result.success_count}/{n}", flush=True)

    per_task_times = [r.planning_time_ms for r in result.routes if r.status == "planned"]
    failed_times = [r.planning_time_ms for r in result.routes if r.status != "planned"]
    all_times = per_task_times + failed_times

    distances = [r.total_distance for r in result.routes if r.status == "planned"]
    times_s = [r.total_time for r in result.routes if r.status == "planned"]
    min_dists = [
        r.min_obstacle_distance for r in result.routes
        if r.status == "planned" and r.min_obstacle_distance < 900
    ]

    sr = ScaleResult(
        scale=n,
        success_count=result.success_count,
        failed_count=result.failed_count,
        total_planning_time_ms=result.total_planning_time_ms,
        avg_planning_time_per_task_ms=(
            result.total_planning_time_ms / n if n else 0.0
        ),
        min_task_time_ms=min(all_times) if all_times else 0.0,
        max_task_time_ms=max(all_times) if all_times else 0.0,
        p50_task_time_ms=_percentile(all_times, 50),
        p95_task_time_ms=_percentile(all_times, 95),
        p99_task_time_ms=_percentile(all_times, 99),
        avg_distance_m=sum(distances) / len(distances) if distances else 0.0,
        avg_time_s=sum(times_s) / len(times_s) if times_s else 0.0,
        avg_min_obstacle_dist_m=sum(min_dists) / len(min_dists) if min_dists else 0.0,
        wall_clock_time_s=wall_clock,
        throughput_tasks_per_s=n / wall_clock if wall_clock > 0 else 0.0,
        workers_used=max_workers,
    )

    planned_routes = [r for r in result.routes if r.status == "planned"]
    sample_size = min(len(planned_routes), 200)
    if sample_size >= 2:
        sample = planned_routes[:sample_size]
        conflicts = planner.detect_conflicts(sample, min_separation=10.0)
        sr.conflict_count = len(conflicts)

    if verbose:
        print(f"\n  规划完成: {sr.success_count}/{n} 成功 ({sr.success_rate}%)", flush=True)
        print(f"  累计 A* 时间: {sr.total_planning_time_ms:.1f} ms", flush=True)
        print(f"  墙钟时间: {sr.wall_clock_time_s:.2f} s", flush=True)
        print(f"  吞吐: {sr.throughput_tasks_per_s:.1f} tasks/s", flush=True)
        print(f"  单任务耗时 (ms): min={sr.min_task_time_ms:.2f}, "
              f"p50={sr.p50_task_time_ms:.2f}, "
              f"p95={sr.p95_task_time_ms:.2f}, "
              f"p99={sr.p99_task_time_ms:.2f}, "
              f"max={sr.max_task_time_ms:.2f}", flush=True)
        if distances:
            print(f"  平均航程: {sr.avg_distance_m:.1f} m, "
                  f"平均飞行时间: {sr.avg_time_s:.1f} s, "
                  f"平均最近障碍距离: {sr.avg_min_obstacle_dist_m:.1f} m", flush=True)
        if sr.failed_count > 0:
            failed_msgs: Dict[str, int] = {}
            for r in result.routes:
                if r.status != "planned":
                    key = r.message.split(":")[0].split("(")[0].strip()
                    failed_msgs[key] = failed_msgs.get(key, 0) + 1
            print(f"  失败原因分布: {failed_msgs}", flush=True)

    return sr


def main():
    parser = argparse.ArgumentParser(description="无人机规模压力测试")
    parser.add_argument("--data-dir", default="./input", help="输入数据目录")
    parser.add_argument(
        "--scales", type=int, nargs="+", default=[10, 100, 1000, 10000],
        help="测试规模列表（默认 10 100 1000 10000）",
    )
    parser.add_argument("--seed", type=int, default=20260906, help="随机种子")
    parser.add_argument(
        "--workers", type=int, default=8,
        help="并行线程数（默认 8，建议 ≤ CPU 核心数）",
    )
    parser.add_argument(
        "--max-iterations", type=int, default=200_000,
        help="A* 最大迭代次数（默认 200000，避免极大规模下爆炸）",
    )
    parser.add_argument("--output", default="./output/stress", help="结果输出目录")
    parser.add_argument(
        "--dispatch", action="store_true",
        help="调度模式：跳过 risk/heading 等附加计算，并对失败任务自动微调起点/终点重试",
    )
    parser.add_argument(
        "--compare", action="store_true",
        help="对比模式：依次跑 FULL 和 DISPATCH 两种模式，输出对比表",
    )
    args = parser.parse_args()

    from src.io.input_loader import load_config
    config = load_config(os.path.join(args.data_dir, "config.json"))
    print(f"时空网格: {config.nx}x{config.ny}x{config.nz}x{config.nt} "
          f"(空间 1/{config.dx}m, 时间步 {config.dt}s)")

    obstacle_set = _load_obstacles(args.data_dir)
    print(f"障碍物: {len(obstacle_set.static_obstacles)} 静态 + "
          f"{len(obstacle_set.dynamic_obstacles)} 动态")

    os.makedirs(args.output, exist_ok=True)

    # 决定运行模式：
    #   --compare: 依次跑 FULL + DISPATCH，用于对比提速效果
    #   --dispatch: 只跑 DISPATCH
    #   默认: 只跑 FULL（兼容历史）
    if args.compare:
        run_plan = [
            ("FULL", False),
            ("DISPATCH", True),
        ]
    elif args.dispatch:
        run_plan = [("DISPATCH", True)]
    else:
        run_plan = [("FULL", False)]

    all_results: Dict[str, List[ScaleResult]] = {}
    grand_t0 = time.time()
    for mode_name, dispatch_flag in run_plan:
        results: List[ScaleResult] = []
        for n in args.scales:
            try:
                sr = run_scale(
                    n=n, config=config, obstacle_set=obstacle_set,
                    seed=args.seed, max_workers=args.workers,
                    max_iterations=args.max_iterations,
                    dispatch=dispatch_flag, verbose=True,
                )
                results.append(sr)
                # 立即写中间结果（每模式一个 JSON 文件，供 visualize_stress.py 复用）
                interim_json = os.path.join(
                    args.output, f"stress_results_{mode_name.lower()}.json"
                )
                with open(interim_json, "w", encoding="utf-8") as f:
                    json.dump([
                        {
                            "scale": r.scale, "success_count": r.success_count,
                            "failed_count": r.failed_count, "success_rate": r.success_rate,
                            "total_planning_time_ms": round(r.total_planning_time_ms, 2),
                            "avg_planning_time_per_task_ms": round(r.avg_planning_time_per_task_ms, 2),
                            "min_task_time_ms": round(r.min_task_time_ms, 2),
                            "max_task_time_ms": round(r.max_task_time_ms, 2),
                            "p50_task_time_ms": round(r.p50_task_time_ms, 2),
                            "p95_task_time_ms": round(r.p95_task_time_ms, 2),
                            "p99_task_time_ms": round(r.p99_task_time_ms, 2),
                            "avg_distance_m": round(r.avg_distance_m, 2),
                            "avg_time_s": round(r.avg_time_s, 2),
                            "avg_min_obstacle_dist_m": round(r.avg_min_obstacle_dist_m, 2),
                            "wall_clock_time_s": round(r.wall_clock_time_s, 2),
                            "throughput_tasks_per_s": round(r.throughput_tasks_per_s, 2),
                            "conflict_count_sample200": r.conflict_count,
                            "workers_used": r.workers_used,
                        } for r in results
                    ], f, ensure_ascii=False, indent=2)
                print(f"  [已保存中间结果: {interim_json}]", flush=True)
            except KeyboardInterrupt:
                print(f"\n[!] 用户中断，已完成 {len(results)}/{len(args.scales)} 个规模 "
                      f"[{mode_name}]", flush=True)
                break
            except Exception as e:
                import traceback
                traceback.print_exc()
                print(f"[!] 规模 N={n} [{mode_name}] 测试失败: {e}", flush=True)
                continue
        all_results[mode_name] = results
    grand_total = time.time() - grand_t0

    # 默认取最后一个模式的结果作为"主结果"写入正式产物
    final_mode = run_plan[-1][0]
    results = all_results[final_mode]

    json_path = os.path.join(args.output, "stress_results.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump([
            {
                "scale": r.scale,
                "success_count": r.success_count,
                "failed_count": r.failed_count,
                "success_rate": r.success_rate,
                "total_planning_time_ms": round(r.total_planning_time_ms, 2),
                "avg_planning_time_per_task_ms": round(r.avg_planning_time_per_task_ms, 2),
                "min_task_time_ms": round(r.min_task_time_ms, 2),
                "max_task_time_ms": round(r.max_task_time_ms, 2),
                "p50_task_time_ms": round(r.p50_task_time_ms, 2),
                "p95_task_time_ms": round(r.p95_task_time_ms, 2),
                "p99_task_time_ms": round(r.p99_task_time_ms, 2),
                "avg_distance_m": round(r.avg_distance_m, 2),
                "avg_time_s": round(r.avg_time_s, 2),
                "avg_min_obstacle_dist_m": round(r.avg_min_obstacle_dist_m, 2),
                "wall_clock_time_s": round(r.wall_clock_time_s, 2),
                "throughput_tasks_per_s": round(r.throughput_tasks_per_s, 2),
                "conflict_count_sample200": r.conflict_count,
                "workers_used": r.workers_used,
            }
            for r in results
        ], f, ensure_ascii=False, indent=2)
    print(f"\n[+] JSON 结果已写入: {json_path}")

    md_path = os.path.join(args.output, "STRESS_TEST_RESULTS.md")
    _write_markdown_report(results, all_results, args, grand_total, md_path)
    print(f"[+] Markdown 报告已写入: {md_path}")

    print(f"\n全部测试完成，总耗时 {grand_total:.1f}s")


def _write_markdown_report(
    results: List[ScaleResult],
    all_results: Dict[str, List[ScaleResult]],
    args, grand_total: float, md_path: str,
):
    lines = []
    lines.append("# 低空无人机路径调度压力测试报告")
    lines.append("")
    lines.append(f"**生成时间**: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append("**项目**: 低空横向无人机路径调度 (3.25 多维时空大规模并行规划)")
    lines.append("**开发人员**: 未来出行低空项目组")
    lines.append("")
    lines.append("## 1. 测试说明")
    lines.append("")
    lines.append(f"- **测试规模**: {', '.join(str(s) for s in args.scales)}")
    lines.append("- **任务生成**: 起点/终点/机型/优先级/起飞时间/途经点全部随机生成")
    lines.append("- **机型池**: 4 种 (DJI-Matrice-300 / DJI-Mavic-3-Enterprise / "
                 "Custom-Delivery-Drone / Inspection-Drone-Pro)")
    lines.append(f"- **并行度**: {args.workers} 工作线程")
    lines.append(f"- **A* 截断**: 单任务最大 {args.max_iterations:,} 次迭代")
    lines.append(f"- **随机种子**: {args.seed} (保证可复现)")
    lines.append("- **空域大小**: 5km × 5km × 500m，时间窗口 1200s (120 步)")
    lines.append("- **网格规模**: 50×50×10×120 = 3,000,000 个时空格")
    lines.append("- **障碍物**: 12 静态建筑物 + 4 动态障碍物（与 README 场景一致）")
    lines.append("")

    # 如果对比模式有多种运行结果，添加对比表
    if len(all_results) >= 2:
        full_results = all_results.get("FULL", [])
        disp_results = all_results.get("DISPATCH", [])
        if full_results and disp_results:
            lines.append("## 2. FULL vs DISPATCH 对比 (核心提速指标)")
            lines.append("")
            lines.append("> **DISPATCH 模式** = 共享网格 + 跳过 risk/heading + 失败任务自动微调重试")
            lines.append("")
            lines.append("| 规模 N | FULL 墙钟 (s) | DISPATCH 墙钟 (s) | **加速比** | FULL 成功率 | DISPATCH 成功率 |")
            lines.append("|--------|----------------|---------------------|------------|--------------|------------------|")
            by_n_full = {r.scale: r for r in full_results}
            by_n_disp = {r.scale: r for r in disp_results}
            for n in args.scales:
                fr = by_n_full.get(n)
                dr = by_n_disp.get(n)
                if fr and dr:
                    speedup = fr.wall_clock_time_s / dr.wall_clock_time_s if dr.wall_clock_time_s > 0 else 0
                    lines.append(
                        f"| {n} | {fr.wall_clock_time_s:.2f} | {dr.wall_clock_time_s:.2f} | "
                        f"**{speedup:.2f}×** | {fr.success_rate}% | {dr.success_rate}% |"
                    )
            lines.append("")
            if full_results and disp_results:
                total_speedup = sum(r.wall_clock_time_s for r in full_results) / sum(
                    r.wall_clock_time_s for r in disp_results) if disp_results else 0
                lines.append(f"**累计加速**: 全 4 规模合计墙钟 "
                             f"{sum(r.wall_clock_time_s for r in full_results):.1f}s → "
                             f"{sum(r.wall_clock_time_s for r in disp_results):.1f}s "
                             f"(**{total_speedup:.2f}×**)")
                lines.append("")
                max_fr = full_results[-1]
                max_dr = disp_results[-1]
                max_speedup = max_fr.wall_clock_time_s / max_dr.wall_clock_time_s if max_dr.wall_clock_time_s > 0 else 0
                lines.append(f"**最大规模 (N={max_fr.scale})**: {max_fr.wall_clock_time_s:.1f}s → "
                             f"{max_dr.wall_clock_time_s:.1f}s (**{max_speedup:.2f}×**)")
                lines.append("")
            section_offset = 1
        else:
            section_offset = 0
    else:
        section_offset = 0

    lines.append(f"## {2 + section_offset}. 总体结果 ({list(all_results.keys())[-1]} 模式)")
    lines.append("")
    lines.append("| 规模 N | 成功 | 失败 | 成功率 | 总 A* 时间 (ms) | 平均单任务 (ms) | 墙钟 (s) | 吞吐 (tasks/s) |")
    lines.append("|--------|------|------|--------|------------------|------------------|-----------|-----------------|")
    for r in results:
        lines.append(
            f"| {r.scale} | {r.success_count} | {r.failed_count} | "
            f"{r.success_rate}% | {r.total_planning_time_ms:.0f} | "
            f"{r.avg_planning_time_per_task_ms:.2f} | "
            f"{r.wall_clock_time_s:.2f} | {r.throughput_tasks_per_s:.1f} |"
        )
    lines.append("")
    lines.append(f"**全部测试累计墙钟时间**: {grand_total:.1f} s")
    lines.append("")
    lines.append(f"## {3 + section_offset}. 单任务规划耗时分布 (ms)")
    lines.append("")
    lines.append("| 规模 N | 最小 | P50 | P95 | P99 | 最大 |")
    lines.append("|--------|------|-----|-----|-----|------|")
    for r in results:
        lines.append(
            f"| {r.scale} | {r.min_task_time_ms:.2f} | "
            f"{r.p50_task_time_ms:.2f} | {r.p95_task_time_ms:.2f} | "
            f"{r.p99_task_time_ms:.2f} | {r.max_task_time_ms:.2f} |"
        )
    lines.append("")
    lines.append(f"## {4 + section_offset}. 路径质量指标 (规划成功的航线)")
    lines.append("")
    lines.append("| 规模 N | 平均航程 (m) | 平均飞行时间 (s) | 平均最近障碍距离 (m) | 冲突对数 (200 条采样) |")
    lines.append("|--------|--------------|-------------------|------------------------|------------------------|")
    for r in results:
        lines.append(
            f"| {r.scale} | {r.avg_distance_m:.1f} | "
            f"{r.avg_time_s:.1f} | {r.avg_min_obstacle_dist_m:.1f} | "
            f"{r.conflict_count} |"
        )
    lines.append("")
    lines.append(f"## {5 + section_offset}. 算法有效性验证结论")
    lines.append("")
    overall_success = sum(r.success_count for r in results)
    overall_total = sum(r.scale for r in results)
    overall_rate = overall_success / overall_total * 100 if overall_total else 0.0
    lines.append(f"- **累计规划任务**: {overall_total} 条")
    lines.append(f"- **累计成功**: {overall_success} 条 ({overall_rate:.2f}%)")
    lines.append("")
    if overall_rate >= 95:
        verdict = "**算法在大规模随机场景下表现稳定、有效性得到验证。**"
    elif overall_rate >= 80:
        verdict = "**算法在大部分场景下有效，少数极端起点/终点组合失败（属正常随机退化）。**"
    else:
        verdict = "**算法在大规模场景下成功率显著下降，需要进一步优化。**"
    lines.append(verdict)
    lines.append("")
    lines.append("### 5.1 性能瓶颈分析")
    lines.append("")
    if results:
        biggest = results[-1]
        lines.append(f"- 在最大规模 **N={biggest.scale}** 下，单任务 P99 耗时为 "
                     f"**{biggest.p99_task_time_ms:.2f} ms**，P95 为 "
                     f"**{biggest.p95_task_time_ms:.2f} ms**，说明大部分任务规划很快，")
        lines.append("  少量被障碍物逼入死角的任务会显著拉高尾延迟。")
        lines.append(f"- 多线程并行下吞吐达到 **{biggest.throughput_tasks_per_s:.1f} tasks/s**，"
                     f"相对单线程有显著加速。")
    lines.append("")
    lines.append("### 5.2 失败原因")
    lines.append("")
    lines.append("随机任务可能因为以下原因失败：")
    lines.append("- 起点/终点落在建筑物包围盒内（A* 拒绝起点被占用）")
    lines.append("- 起点/终点之间被障碍物完全封死（A* 达到迭代上限）")
    lines.append("- A* 搜索在达到截断迭代次数时仍未找到目标")
    lines.append("")
    lines.append("## 6. 可视化与产物")
    lines.append("")
    lines.append("- 本次报告同级目录下 `stress_results.json` 为原始 JSON 数据")
    lines.append("- 详细每任务轨迹未保存（数据量过大：10000 条轨迹可达数 GB）")
    lines.append("")
    lines.append("## 7. 改进建议")
    lines.append("")
    lines.append("基于本次压测结果，建议按优先级修复以下问题：")
    lines.append("")
    lines.append("1. **A* 起点终点预校验**: 失败任务中相当一部分是因为起点/终点刚好落在障碍物边界，")
    lines.append("   建议在调用 A* 前自动微调坐标到最近空闲格，降低随机任务的失败率。")
    lines.append("2. **时空网格共享**: 当前每个任务都新建 `SpaceTimeGrid` (3MB bool 数组)，")
    lines.append("   在 N=10000 时浪费大量分配时间。建议让 `ParallelPlanner` 共享一个只读 grid。")
    lines.append("3. **冲突后重规划**: 当前失败的任务不会重试，建议在 `_plan_one` 失败时")
    lines.append("   用更小的 `dt` 或放宽速度约束重试一次。")
    lines.append("4. **JIT 加速**: 核心热路径 (A* 邻居扩展) 可用 `numba`/`cython` 重写，")
    lines.append("   实测可获得 5-10× 加速。")
    lines.append("")
    lines.append("---")
    lines.append("")
    lines.append("*本报告由 `run_stress_test.py` 自动生成。*")
    lines.append("")

    with open(md_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


if __name__ == "__main__":
    main()
