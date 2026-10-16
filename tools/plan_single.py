#!/usr/bin/env python3
"""
单机路径规划 (3.26 智能飞行线路规划算法)

对单个无人机执行完整的路径规划：
1. 多源融合加载配置、障碍物、飞行器和任务数据 (3.25 模块三)
2. 验证飞行器是否满足任务需求（载荷、续航、速度、高度）
3. 多维时空 A* 路径规划 (3.25)，带通行代价层
4. 多目标航线优化（航程/时间/风险/能耗）+ 结构化规划解释
5. 路径合法性验证（连续性/障碍/性能/高度/时空占用 7 维）
6. 输出 JSON / Markdown 到 --output-dir

可视化由独立脚本 visualize.py 负责：
    python visualize.py --input output/demo/

用法:
    python plan_single.py                                      # 默认任务 task-001
    python plan_single.py --task task-001                      # 指定任务
    python plan_single.py --input-dir ./input --output-dir ./output/demo
    python plan_single.py --no-optimize                        # 跳过 3.26 优化
    python plan_single.py --no-cost-grid                       # 关闭通行代价层
    python plan_single.py --no-validate                        # 跳过合法性验证
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.models.space import SpaceTimeConfig
from src.models.obstacles import ObstacleSet
from src.models.aircraft import AircraftPerformance, FlightTask
from src.models.task import PlannedRoute, PlanningResult
from src.io.input_loader import (
    load_config,
    load_obstacle_set_fused,
    load_aircraft_performances,
    load_tasks,
)
from src.io.output_writer import (
    write_planning_result,
    write_route,
    write_waypoints,
    write_validation,
    generate_report,
    ensure_output_dir,
)
from src.planners.spacetime_astar import SpaceTimeAStar
from src.planners.route_optimizer import RouteOptimizer
from src.grid.cost import build_clearance_cost_grid
from src.validator import PathValidator

# 编排逻辑已下沉到 SDK（原先定义在本脚本里，造成 uav_planner 反向依赖命令行脚本）
from uav_planner.pipeline import run_single_task as plan_single_task
from tools.planning_archive import archive_planning_result, snapshot_input_files, write_json


def main():
    parser = argparse.ArgumentParser(description="单机路径规划")
    parser.add_argument("--input-dir", default="./input", help="输入数据目录")
    parser.add_argument("--output-dir", default="./output/demo", help="输出目录")
    parser.add_argument("--task", default=None,
                        help="指定任务 ID（默认：按优先级选最高）")
    parser.add_argument("--no-optimize", action="store_true", help="跳过 3.26 多目标优化")
    parser.add_argument("--max-iterations", type=int, default=2_000_000,
                        help="A* 单任务最大迭代次数")
    parser.add_argument("--no-cost-grid", action="store_true",
                        help="关闭通行代价层（默认开启，偏好远离建筑群的航路）")
    parser.add_argument("--clearance", type=float, default=200.0,
                        help="障碍物净空膨胀半径（米），用于构建代价层")
    parser.add_argument("--clearance-cost", type=float, default=3.0,
                        help="净空区内通行代价倍率")
    parser.add_argument("--no-validate", action="store_true",
                        help="跳过路径合法性验证")
    args = parser.parse_args()

    print("=" * 60)
    print("  单机路径规划 (Single-UAV Planning)")
    print("  3.25 多维时空 A* + 3.26 多目标优化")
    print("=" * 60)

    # 1. 加载数据（多源融合）
    print("\n[1/6] 加载输入数据...")
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
    capture_directory = Path(args.output_dir) / "planning_data" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    source_snapshot = snapshot_input_files(args.input_dir, capture_directory / "input", vars(args))

    # 2. 构建通行代价层（与占用网格正交）
    cost_grid = None
    if not args.no_cost_grid:
        from src.models.space import SpaceTimeGrid
        base_grid = SpaceTimeGrid(config)
        cost_grid = build_clearance_cost_grid(
            base_grid, obstacle_set,
            clearance=args.clearance, clearance_cost=args.clearance_cost,
        )
        print(f"  代价层: 净空 {args.clearance:.0f}m × {args.clearance_cost:.1f} 倍率")

    # 3. 选择任务
    if args.task:
        task = next((t for t in tasks if t.task_id == args.task), None)
        if task is None:
            print(f"[!] 任务 {args.task} 不存在")
            sys.exit(1)
    else:
        # 按业务优先级 + 调度优先级选最高
        task = max(tasks, key=lambda t: (
            t.requirements.priority_level,
            t.priority,
        ))
    print(f"\n[2/6] 选中任务: {task.task_id} ({task.task_name})")
    print(f"  飞机: {task.aircraft_id}")
    print(f"  优先级: 调度={task.priority} / 业务={task.requirements.priority_level}")
    print(f"  航点数: {len(task.waypoints)}")

    perf = performances.get(task.aircraft_id)
    if perf is None:
        print(f"[!] 飞行器 {task.aircraft_id} 未在配置中找到")
        sys.exit(1)

    print(f"  机型: {perf.model} ({perf.category})")
    print(f"  自重: {perf.physical.weight_kg}kg / 载荷上限: "
          f"{perf.physical.payload_capacity_kg}kg")
    print(f"  最大速度: {perf.get_max_speed()} m/s / "
          f"续航: {perf.get_max_flight_time()/60:.1f} min")

    # 4. 执行规划
    input_record = write_json(capture_directory / "single_input.json", {
        "format_version": 1, "git_sha": source_snapshot["git_sha"], "arguments": vars(args),
        "config": asdict(config), "obstacles": asdict(obstacle_set),
        "tasks": [asdict(task)], "aircraft_performances": {task.aircraft_id: asdict(perf)},
    })
    print("\n[3/6] 执行规划...")
    final_route, all_routes, warnings = plan_single_task(
        task, config, obstacle_set, perf,
        optimize=not args.no_optimize,
        max_iterations=args.max_iterations,
        cost_grid=cost_grid,
    )

    # 5. 路径合法性验证（3.26 验证模块）
    validation = None
    if not args.no_validate:
        print("\n[4/6] 路径合法性验证...")
        if final_route.status == "planned":
            validator = PathValidator(obstacle_set=obstacle_set)
            validation = validator.validate_route(final_route, performance=perf)
            print(f"  {validation.summary_text()}")
            for issue in validation.issues[:5]:
                print(f"    [{issue.severity.value}] {issue.code}: {issue.message}")
            if len(validation.issues) > 5:
                print(f"    ...（共 {len(validation.issues)} 条）")
        else:
            print("  跳过：规划未成功")
    else:
        print("\n[4/6] 跳过合法性验证")

    # 6. 保存结果
    print("\n[5/6] 保存结果...")
    result = PlanningResult(
        request_id=f"single-{task.task_id}",
        routes=all_routes,
        total_planning_time_ms=0,
        success_count=sum(1 for r in all_routes if r.status == "planned"),
        failed_count=sum(1 for r in all_routes if r.status != "planned"),
    )
    result_file = write_planning_result(result, args.output_dir)
    write_json(capture_directory / "single_manifest.json", {
        "format_version": 1, "source_input_manifest": "input/manifest.json", "input": input_record,
        "final_route_id": final_route.route_id, "warnings": warnings,
        **archive_planning_result(result, capture_directory, "single"),
    })
    print(f"  结果: {result_file}")

    routes_dir = os.path.join(args.output_dir, "routes")
    ensure_output_dir(routes_dir)
    for r in all_routes:
        if r.status == "planned":
            write_route(r, routes_dir)

    # 验证结果
    if validation is not None:
        vfile = write_validation(
            validation, args.output_dir,
            f"validation_{task.task_id}.json",
            extra={"task_id": task.task_id,
                   "aircraft_id": task.aircraft_id,
                   "route_id": final_route.route_id,
                   "data_quality": data_quality},
        )
        print(f"  验证: {vfile}")

    # 航点指令
    if final_route.status == "planned" and not args.no_optimize:
        optimizer = RouteOptimizer(config, obstacle_set, perf, cost_grid=cost_grid)
        waypoints = optimizer.generate_waypoints(final_route.trajectory, min_spacing=80.0)
        wp_file = write_waypoints(waypoints, args.output_dir,
                                  f"waypoints_{task.task_id}.json")
        print(f"  航点: {wp_file} ({len(waypoints)} 个)")

    # 环境快照（供 visualize.py 使用）
    _save_environment(
        static_path,
        dynamic_path,
        os.path.join(args.output_dir, "environment.json"),
        config_path=os.path.join(args.input_dir, "config.json"),
    )

    # 7. 报告
    print("\n[6/6] 生成报告...")
    config_info = {
        "空间范围": f"{config.x_min}-{config.x_max} x {config.y_min}-{config.y_max} x "
                    f"{config.z_min}-{config.z_max} m",
        "空间分辨率": f"{config.dx} x {config.dy} x {config.dz} m",
        "网格维度": f"{config.nx} x {config.ny} x {config.nz}",
        "时间范围": f"{config.t_min}-{config.t_max} s",
        "时间分辨率": f"{config.dt} s",
        "时间步数": f"{config.nt}",
    }
    obstacle_info = obstacle_set.summary()
    report_file = generate_report(
        result, config_info, obstacle_info, args.output_dir,
        validation=validation, data_quality=data_quality,
    )
    print(f"  报告: {report_file}")

    print("\n" + "=" * 60)
    print(f"  完成! 规划状态: {'✓ 成功' if final_route.status == 'planned' else '✗ 失败'}")
    if validation is not None:
        print(f"  合法性: {validation.summary_text()}")
    print(f"  输出: {os.path.abspath(args.output_dir)}")
    print(f"  可视化: python visualize.py --input {args.output_dir}")
    if warnings:
        print(f"  ⚠ {len(warnings)} 个兼容性警告，详见上文")
    print("=" * 60)
    # 有 ERROR 级问题时以非零码退出，便于 CI 拦截
    if validation is not None and validation.has_errors():
        sys.exit(2)


def _save_environment(static_path, dynamic_path, output_path, config_path=None):
    """保存环境快照（供 visualize.py / validate 复现）"""
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
