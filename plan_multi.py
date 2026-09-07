#!/usr/bin/env python3
"""
多机批量路径规划 (3.25 大规模并行规划)

支持多飞行器、多任务的并行路径规划：
1. 加载配置、障碍物、飞行器和任务数据
2. 按调度优先级 + 业务优先级排序任务
3. 多线程并行执行 A* 规划
4. 飞行器间冲突检测（时间对齐 + 最小安全间隔）
5. 输出聚合结果 JSON + Markdown

可视化由 visualize.py / visualize_stress.py 负责：
    python visualize.py --input ./output/demo
    python visualize_stress.py --input ./output/stress

用法:
    python plan_multi.py                                  # 默认全部任务
    python plan_multi.py --input-dir ./input              # 指定输入
    python plan_multi.py --dispatch                       # 调度模式
    python plan_multi.py --workers 8 --scales 10 100 1000 10000  # 压测
    python plan_multi.py --no-validation                  # 跳过需求校验
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from typing import Dict, List, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from src.models.space import SpaceTimeConfig
from src.models.obstacles import ObstacleSet
from src.models.aircraft import AircraftPerformance, FlightTask
from src.models.task import PlannedRoute, PlanningResult
from src.io.input_loader import (
    load_config,
    load_obstacle_set,
    load_aircraft_performances,
    load_tasks,
)
from src.io.output_writer import (
    write_planning_result,
    write_route,
    generate_report,
    ensure_output_dir,
)
from src.planners.parallel_planner import ParallelPlanner


def validate_tasks_with_aircraft(
    tasks: List[FlightTask],
    performances: Dict[str, AircraftPerformance],
) -> Tuple[List[FlightTask], List[Tuple[str, List[str]]]]:
    """
    验证每个任务的飞行器需求兼容性。

    Returns:
        (compatible_tasks, incompatible_pairs)
        - compatible_tasks: 通过校验的任务列表
        - incompatible_pairs: [(task_id, [问题]), ...]
    """
    compatible = []
    incompatible = []
    for task in tasks:
        perf = performances.get(task.aircraft_id)
        if perf is None:
            incompatible.append((task.task_id, [f"未找到飞机 {task.aircraft_id}"]))
            continue
        ok, problems = task.check_aircraft_compatibility(perf)
        if ok:
            compatible.append(task)
        else:
            incompatible.append((task.task_id, problems))
    return compatible, incompatible


def plan_batch(
    tasks: List[FlightTask],
    performances: Dict[str, AircraftPerformance],
    config: SpaceTimeConfig,
    obstacle_set: ObstacleSet,
    dispatch: bool = False,
    workers: int = 4,
    max_iterations: int = 200_000,
) -> PlanningResult:
    """批量并行规划"""
    planner = ParallelPlanner(
        config=config, obstacle_set=obstacle_set,
        max_workers=workers,
        max_iterations=max_iterations,
    )
    return planner.plan_batch(tasks, performances, request_id=f"multi-{len(tasks)}",
                              dispatch=dispatch)


def main():
    parser = argparse.ArgumentParser(description="多机批量路径规划")
    parser.add_argument("--input-dir", default="./input", help="输入数据目录")
    parser.add_argument("--output-dir", default="./output/multi", help="输出目录")
    parser.add_argument("--workers", type=int, default=4, help="并行线程数")
    parser.add_argument("--dispatch", action="store_true",
                        help="调度模式（共享网格 + 跳过 risk/heading + 预微调）")
    parser.add_argument("--max-iterations", type=int, default=200_000,
                        help="A* 单任务最大迭代次数")
    parser.add_argument("--no-validation", action="store_true",
                        help="跳过飞行器需求校验")
    parser.add_argument("--conflict-check", action="store_true",
                        help="规划完成后执行飞行器间冲突检测")
    args = parser.parse_args()

    print("=" * 60)
    print("  多机批量路径规划 (Multi-UAV Planning)")
    print("  3.25 大规模并行规划引擎")
    print("=" * 60)

    # 1. 加载数据
    print("\n[1/5] 加载输入数据...")
    config = load_config(os.path.join(args.input_dir, "config.json"))
    print(f"  时空网格: {config.nx}x{config.ny}x{config.nz} (空间), {config.nt} 时间步")

    obstacle_set = load_obstacle_set(
        os.path.join(args.input_dir, "static_obstacles.json"),
        os.path.join(args.input_dir, "dynamic_obstacles.json"),
    )
    print(f"  障碍物: {len(obstacle_set.static_obstacles)} 静态 + "
          f"{len(obstacle_set.dynamic_obstacles)} 动态")

    performances = load_aircraft_performances(
        os.path.join(args.input_dir, "aircraft_config.json")
    )
    print(f"  飞行器: {len(performances)} 架")

    tasks = load_tasks(os.path.join(args.input_dir, "tasks.json"))
    print(f"  任务: {len(tasks)} 个")

    ensure_output_dir(args.output_dir)

    # 2. 校验
    if not args.no_validation:
        print("\n[2/5] 飞行器需求校验...")
        compatible, incompatible = validate_tasks_with_aircraft(tasks, performances)
        if incompatible:
            print(f"  ⚠ {len(incompatible)} 个任务飞行器不兼容:")
            for tid, problems in incompatible:
                print(f"    [{tid}]:")
                for p in problems:
                    print(f"      - {p}")
            print(f"  → 仅规划 {len(compatible)} 个兼容任务")
            tasks = compatible
        else:
            print(f"  ✓ 全部 {len(tasks)} 个任务飞行器兼容")
    else:
        print("\n[2/5] 跳过校验")

    if not tasks:
        print("[!] 无可规划任务")
        sys.exit(1)

    # 3. 执行规划
    mode = "DISPATCH" if args.dispatch else "FULL"
    print(f"\n[3/5] 并行规划 ({mode}, workers={args.workers})...")
    t0 = time.time()
    result = plan_batch(
        tasks, performances, config, obstacle_set,
        dispatch=args.dispatch, workers=args.workers,
        max_iterations=args.max_iterations,
    )
    elapsed = time.time() - t0
    print(f"  成功 {result.success_count}/{len(tasks)}  ({result.success_count/len(tasks)*100:.1f}%)")
    print(f"  A* 总耗时: {result.total_planning_time_ms:.1f} ms")
    print(f"  墙钟: {elapsed:.2f} s ({len(tasks)/elapsed:.1f} tasks/s)")

    # 失败原因分布
    failed_routes = [r for r in result.routes if r.status != "planned"]
    if failed_routes:
        from collections import Counter
        reasons = Counter()
        for r in failed_routes:
            key = r.message.split(":")[0].split("(")[0].strip()
            reasons[key] += 1
        print(f"  失败原因:")
        for reason, count in reasons.most_common():
            print(f"    {count}× {reason}")

    # 4. 冲突检测
    if args.conflict_check:
        print("\n[4/5] 飞行器间冲突检测...")
        planner = ParallelPlanner(config=config, obstacle_set=obstacle_set,
                                   max_workers=1)
        planned = [r for r in result.routes if r.status == "planned"]
        # 采样最多 200 条做冲突检测，避免 O(n²) 爆炸
        sample = planned[:200]
        conflicts = planner.detect_conflicts(sample, min_separation=10.0)
        print(f"  采样 {len(sample)} 条规划航线，发现 {len(conflicts)} 对冲突")
        if conflicts:
            conflict_log = os.path.join(args.output_dir, "conflicts.json")
            with open(conflict_log, "w", encoding="utf-8") as f:
                json.dump(conflicts, f, ensure_ascii=False, indent=2)
            print(f"  冲突详情: {conflict_log}")
    else:
        print("\n[4/5] 跳过冲突检测")

    # 5. 保存结果
    print("\n[5/5] 保存结果...")
    result_file = write_planning_result(result, args.output_dir)
    print(f"  结果: {result_file}")

    routes_dir = os.path.join(args.output_dir, "routes")
    ensure_output_dir(routes_dir)
    for r in result.routes:
        if r.status == "planned":
            write_route(r, routes_dir)

    # 环境快照
    _save_environment(
        os.path.join(args.input_dir, "config.json"),
        os.path.join(args.input_dir, "static_obstacles.json"),
        os.path.join(args.input_dir, "dynamic_obstacles.json"),
        os.path.join(args.output_dir, "environment.json"),
    )

    # 报告
    config_info = {
        "空间范围": f"{config.x_min}-{config.x_max} x {config.y_min}-{config.y_max} x "
                    f"{config.z_min}-{config.z_max} m",
        "空间分辨率": f"{config.dx} x {config.dy} x {config.dz} m",
        "网格维度": f"{config.nx} x {config.ny} x {config.nz}",
        "时间范围": f"{config.t_min}-{config.t_max} s",
        "时间分辨率": f"{config.dt} s",
        "时间步数": f"{config.nt}",
    }
    report_file = generate_report(result, config_info, obstacle_set.summary(), args.output_dir)
    print(f"  报告: {report_file}")

    print("\n" + "=" * 60)
    print(f"  完成! 规划 {result.success_count}/{len(tasks)} 成功")
    print(f"  输出: {os.path.abspath(args.output_dir)}")
    print(f"  可视化: python visualize.py --input {args.output_dir}")
    print("=" * 60)


def _save_environment(config_path, static_path, dynamic_path, output_path):
    snap = {}
    for label, path in [("config", config_path), ("static_obstacles", static_path),
                        ("dynamic_obstacles", dynamic_path)]:
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as f:
                snap[label] = json.load(f)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(snap, f, ensure_ascii=False, indent=2)


if __name__ == "__main__":
    main()