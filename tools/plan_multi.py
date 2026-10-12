#!/usr/bin/env python3
"""
多机批量路径规划 (3.25 大规模并行规划)

支持多飞行器、多任务的并行路径规划：
1. 多源融合加载配置、障碍物、飞行器和任务数据
2. 按调度优先级 + 业务优先级排序任务
3. 多线程并行执行 A* 规划（共享占用网格 + 可选通行代价层）
4. 时空占用表冲突检测 + 冲突协调建议
5. 路径合法性验证（连续性/障碍/性能/高度/多机间隔）
6. 输出聚合结果 JSON + Markdown

可视化由 visualize.py / visualize_stress.py 负责：
    python visualize.py --input ./output/demo
    python visualize_stress.py --input ./output/stress

用法:
    python plan_multi.py                                  # 默认全部任务
    python plan_multi.py --input-dir ./input              # 指定输入
    python plan_multi.py --dispatch                       # 调度模式
    python plan_multi.py --workers 8                      # 线程数
    python plan_multi.py --no-validation                  # 跳过需求校验
    python plan_multi.py --conflict-check                 # 时空占用表冲突检测
    python plan_multi.py --validate                       # 路径合法性验证
    python plan_multi.py --cost-grid                      # 启用通行代价层
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from typing import Dict, List, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.models.space import SpaceTimeConfig
from src.models.obstacles import ObstacleSet
from src.models.aircraft import AircraftPerformance, FlightTask
from src.models.task import PlanningResult
from src.io.input_loader import (
    load_config,
    load_obstacle_set_fused,
    load_aircraft_performances,
    load_tasks,
)
from src.io.output_writer import (
    write_planning_result,
    write_route,
    write_validation,
    generate_report,
    ensure_output_dir,
)
from src.planners.parallel_planner import ParallelPlanner
from src.validator import PathValidator

# 编排逻辑已下沉到 SDK（原先定义在本脚本里，造成 uav_planner 反向依赖命令行脚本）
from uav_planner.pipeline import (
    run_batch as plan_batch,
    validate_tasks_with_aircraft,
)


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
    parser.add_argument("--no-avoid-conflicts", action="store_true",
                        help="关闭多机冲突避免（任务书 §五.4 默认开启；"
                             "仅用于性能基准对比，结果可能出现同格同时重叠）")
    parser.add_argument("--conflict-check", action="store_true",
                        help="额外输出残余冲突详情 + 协调建议到 conflicts.json")
    parser.add_argument("--validate", action="store_true",
                        help="执行路径合法性验证（含多机间隔检查）")
    parser.add_argument("--no-cost-grid", action="store_true",
                        help="关闭通行代价层（默认开启）")
    parser.add_argument("--clearance", type=float, default=200.0,
                        help="障碍物净空膨胀半径（米）")
    parser.add_argument("--clearance-cost", type=float, default=3.0,
                        help="净空区内通行代价倍率")
    parser.add_argument("--min-separation", type=float, default=10.0,
                        help="多机最小安全间隔（米）")
    parser.add_argument("--conflict-sample", type=int, default=0,
                        help="冲突检测最多检查多少条航线（0 = 全量；"
                             "超限时随机抽样，抽样不能用于安全放行判定）")
    args = parser.parse_args()

    print("=" * 60)
    print("  多机批量路径规划 (Multi-UAV Planning)")
    print("  3.25 大规模并行规划引擎")
    print("=" * 60)

    # 1. 加载数据（多源融合）
    print("\n[1/5] 加载输入数据...")
    config = load_config(os.path.join(args.input_dir, "config.json"))
    print(f"  时空网格: {config.nx}x{config.ny}x{config.nz} (空间), {config.nt} 时间步")

    static_path = os.path.join(args.input_dir, "static_obstacles.json")
    dynamic_path = os.path.join(args.input_dir, "dynamic_obstacles.json")
    obstacle_set, data_quality = load_obstacle_set_fused(
        [static_path], [dynamic_path]
    )
    print(f"  障碍物: {len(obstacle_set.static_obstacles)} 静态 + "
          f"{len(obstacle_set.dynamic_obstacles)} 动态")
    print(f"  融合数据源: {data_quality['registered_sources']} 个 "
          f"(告警 {data_quality['warning_count']} 条)")
    for w in data_quality["warnings"]:
        print(f"    ⚠ {w}")

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
    use_cost_grid = not args.no_cost_grid
    avoidance = not args.no_avoid_conflicts
    print(f"\n[3/5] 并行规划 ({mode}, workers={args.workers}, "
          f"代价层={'on' if use_cost_grid else 'off'}, "
          f"冲突避免={'on' if avoidance else 'OFF'})...")
    t0 = time.time()
    result, planner = plan_batch(
        tasks, performances, config, obstacle_set,
        dispatch=args.dispatch, workers=args.workers,
        max_iterations=args.max_iterations,
        use_cost_grid=use_cost_grid,
        conflict_avoidance=avoidance,
        min_separation=args.min_separation,
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

    # 4. 多机冲突状态（规划阶段已做避免；这里如实报告残余冲突）
    validation = None
    planned = [r for r in result.routes if r.status == "planned"]
    conflicts = list(result.conflicts)

    print(f"\n[4/5] 多机冲突状态（安全间隔 {args.min_separation}m）...")
    if avoidance:
        print(f"  冲突避免: 已开启（规划闭环内消化）"
              if not conflicts else
              f"  ⚠ 冲突避免后仍有 {len(conflicts)} 对冲突未消解")
    else:
        print(f"  冲突避免: 已关闭（--no-avoid-conflicts）")

    if args.conflict_sample and len(planned) > args.conflict_sample:
        # 显式要求抽样时才抽；抽样结果不能用于安全放行
        conflicts = planner.detect_conflicts_occupancy(
            planned, min_separation=args.min_separation,
            max_routes=args.conflict_sample,
        )
        print(f"  （抽样检查 {args.conflict_sample} 条航线）")

    if args.conflict_check or conflicts:
        if conflicts:
            resolved = planner.resolve_conflicts(conflicts, planned)
            by_resolution: Dict[str, int] = {}
            for c in resolved:
                by_resolution[c["resolution"]] = by_resolution.get(c["resolution"], 0) + 1
            for kind, count in by_resolution.items():
                print(f"    建议{kind}: {count} 条")
        else:
            resolved = []
            print("  ✓ 无残余冲突")
        conflict_log = os.path.join(args.output_dir, "conflicts.json")
        with open(conflict_log, "w", encoding="utf-8") as f:
            json.dump({
                "min_separation_m": args.min_separation,
                "avoidance_enabled": avoidance,
                "n_planned_routes": len(planned),
                "n_residual_conflicts": len(resolved),
                "conflicts": resolved,
            }, f, ensure_ascii=False, indent=2)
        print(f"  冲突详情 + 协调建议: {conflict_log}")
    elif not conflicts:
        print("  ✓ 无残余冲突")

    if args.validate:
        print("\n      路径合法性验证...")
        validator = PathValidator(
            obstacle_set=obstacle_set,
            min_separation=args.min_separation,
            config=config,  # → 多机间隔校验走线性复杂度的占用表路径
        )
        validation = validator.validate_multi_routes(planned)
        # 单机性能/高度/连续性检查（按 aircraft 找对应性能）
        for route in planned:
            perf = performances.get(route.aircraft_id)
            if perf is not None:
                validation.merge(validator.validate_route(route, performance=perf))
        print(f"  {validation.summary_text()}")
        for issue in validation.issues[:5]:
            print(f"    [{issue.severity.value}] {issue.code}: {issue.message}")
        if len(validation.issues) > 5:
            print(f"    ...（共 {len(validation.issues)} 条）")
    else:
        print("      跳过路径合法性验证（--validate 开启）")

    # 5. 保存结果
    print("\n[5/5] 保存结果...")
    result_file = write_planning_result(result, args.output_dir)
    print(f"  结果: {result_file}")

    routes_dir = os.path.join(args.output_dir, "routes")
    ensure_output_dir(routes_dir)
    for r in result.routes:
        if r.status == "planned":
            write_route(r, routes_dir)

    if validation is not None:
        vfile = write_validation(
            validation, args.output_dir, "validation.json",
            extra={"request_id": result.request_id,
                   "n_planned_routes": len(planned),
                   "data_quality": data_quality},
        )
        print(f"  验证: {vfile}")

    # 环境快照
    _save_environment(
        static_path,
        dynamic_path,
        os.path.join(args.output_dir, "environment.json"),
        config_path=os.path.join(args.input_dir, "config.json"),
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
    report_file = generate_report(
        result, config_info, obstacle_set.summary(), args.output_dir,
        validation=validation, data_quality=data_quality,
    )
    print(f"  报告: {report_file}")

    print("\n" + "=" * 60)
    print(f"  完成! 规划 {result.success_count}/{len(tasks)} 成功")
    if validation is not None:
        print(f"  合法性: {validation.summary_text()}")
    if conflicts:
        print(f"  ✗ 多机冲突: 仍有 {len(conflicts)} 对未消解"
              f"（间隔要求 {args.min_separation}m）")
    else:
        print(f"  ✓ 多机冲突: 0 对（间隔要求 {args.min_separation}m）")
    print(f"  输出: {os.path.abspath(args.output_dir)}")
    print(f"  可视化: python visualize.py --input {args.output_dir}")
    print("=" * 60)

    # 退出码约定：
    #   0 = 全部成功且无冲突、无验证错误
    #   2 = 存在残余多机冲突，或合法性验证报 ERROR
    # 残余冲突即使没开 --validate 也必须让调用方感知——否则 CI 会把
    # "两机在同一格同一秒重叠" 当成成功。
    if conflicts:
        sys.exit(2)
    if validation is not None and validation.has_errors():
        sys.exit(2)


def _save_environment(static_path, dynamic_path, output_path, config_path=None):
    snap = {}
    for label, path in [("config", config_path), ("static_obstacles", static_path),
                        ("dynamic_obstacles", dynamic_path)]:
        if path and os.path.exists(path):
            with open(path, "r", encoding="utf-8") as f:
                snap[label] = json.load(f)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(snap, f, ensure_ascii=False, indent=2)


if __name__ == "__main__":
    main()