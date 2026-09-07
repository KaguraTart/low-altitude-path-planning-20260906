#!/usr/bin/env python3
"""
单机路径规划 (3.26 智能飞行线路规划算法)

对单个无人机执行完整的路径规划：
1. 加载配置、障碍物、飞行器和任务数据
2. 验证飞行器是否满足任务需求（载荷、续航、速度、高度）
3. 多维时空 A* 路径规划 (3.25)
4. 多目标航线优化（航程/时间/风险/能耗）
5. 输出 JSON / Markdown 到 --output-dir

可视化由独立脚本 visualize.py 负责：
    python visualize.py --input output/demo/

用法:
    python plan_single.py                                      # 默认任务 task-001
    python plan_single.py --task task-001                      # 指定任务
    python plan_single.py --input-dir ./input --output-dir ./output/demo
    python plan_single.py --no-optimize                        # 跳过 3.26 优化
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

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
    write_waypoints,
    generate_report,
    ensure_output_dir,
)
from src.planners.spacetime_astar import SpaceTimeAStar
from src.planners.route_optimizer import RouteOptimizer


def plan_single_task(
    task: FlightTask,
    config: SpaceTimeConfig,
    obstacle_set: ObstacleSet,
    perf: AircraftPerformance,
    optimize: bool = True,
    max_iterations: int = 2_000_000,
) -> tuple[PlannedRoute, list[PlannedRoute], list[str]]:
    """
    对单个任务执行完整规划。

    Returns:
        (final_route, all_routes, warnings)
        - final_route: 优化后的最终航线
        - all_routes: A* + 优化所有中间结果
        - warnings: 飞行器需求校验警告
    """
    warnings = []

    # 1. 任务 + 飞行器兼容性校验
    valid, msg = task.validate()
    if not valid:
        raise ValueError(f"任务定义非法: {msg}")
    compatible, problems = task.check_aircraft_compatibility(perf)
    if not compatible:
        warnings.extend(problems)
        print(f"[!] 飞行器 {perf.aircraft_id} 与任务 {task.task_id} 不兼容:")
        for p in problems:
            print(f"    - {p}")

    # 2. A* 规划
    print(f"\n[1/2] 时空 A* 规划 ({task.task_id})...")
    planner = SpaceTimeAStar(
        config=config,
        obstacle_set=obstacle_set,
        max_speed=perf.get_max_speed(),
        max_iterations=max_iterations,
    )

    all_routes = []
    t0 = time.time()

    if len(task.waypoints) > 2:
        # 多航点分段规划
        traj = []
        current_time = task.departure_time
        success = True
        for seg_idx in range(len(task.waypoints) - 1):
            wp_s = task.waypoints[seg_idx]
            wp_e = task.waypoints[seg_idx + 1]
            seg_route = planner.plan(
                start=(wp_s.x, wp_s.y, wp_s.z),
                goal=(wp_e.x, wp_e.y, wp_e.z),
                start_time=current_time,
                task_id=task.task_id,
                aircraft_id=task.aircraft_id,
                route_id=f"astar-{task.task_id}-seg{seg_idx}",
            )
            if seg_route.status != "planned":
                success = False
                print(f"    航段 {seg_idx} 失败: {seg_route.message}")
                break
            if traj:
                traj.extend(seg_route.trajectory[1:])
            else:
                traj.extend(seg_route.trajectory)
            current_time = seg_route.trajectory[-1].t + wp_e.hover_time

        if success:
            route = PlannedRoute(
                route_id=f"astar-{task.task_id}",
                task_id=task.task_id,
                aircraft_id=task.aircraft_id,
                trajectory=traj,
                status="planned",
                algorithm="SpaceTimeAStar-MultiWaypoint",
            )
            route.compute_metrics(perf)
            all_routes.append(route)
            print(f"    成功: {route.total_distance:.1f}m, {route.total_time:.1f}s, "
                  f"{len(traj)}点")
        else:
            failed = PlannedRoute(
                route_id=f"astar-{task.task_id}",
                task_id=task.task_id,
                aircraft_id=task.aircraft_id,
                status="failed",
                message="多航点规划失败",
            )
            all_routes.append(failed)
    else:
        route = planner.plan(
            start=(task.start.x, task.start.y, task.start.z),
            goal=(task.end.x, task.end.y, task.end.z),
            start_time=task.departure_time,
            task_id=task.task_id,
            aircraft_id=task.aircraft_id,
            route_id=f"astar-{task.task_id}",
        )
        all_routes.append(route)
        if route.status == "planned":
            print(f"    成功: {route.total_distance:.1f}m, {route.total_time:.1f}s")
        else:
            print(f"    失败: {route.message}")

    final_route = all_routes[-1]

    # 3. 多目标优化
    if optimize and final_route.status == "planned":
        print(f"\n[2/2] 多目标航线优化 ({task.task_id})...")
        optimizer = RouteOptimizer(config, obstacle_set, perf)
        optimized = optimizer.optimize(task, route_id=f"opt-{task.task_id}")
        all_routes.append(optimized)
        final_route = optimized
        print(f"    优化完成: {final_route.total_distance:.1f}m, "
              f"{final_route.total_time:.1f}s")
        print(f"      风险: max={final_route.max_risk:.4f}, "
              f"avg={final_route.avg_risk:.4f}, "
              f"最近障碍={final_route.min_obstacle_distance:.1f}m")
    else:
        print(f"\n[2/2] 跳过优化")

    print(f"\n  规划耗时: {(time.time()-t0)*1000:.1f} ms")
    return final_route, all_routes, warnings


def main():
    parser = argparse.ArgumentParser(description="单机路径规划")
    parser.add_argument("--input-dir", default="./input", help="输入数据目录")
    parser.add_argument("--output-dir", default="./output/demo", help="输出目录")
    parser.add_argument("--task", default=None,
                        help="指定任务 ID（默认：按优先级选最高）")
    parser.add_argument("--no-optimize", action="store_true", help="跳过 3.26 多目标优化")
    parser.add_argument("--max-iterations", type=int, default=2_000_000,
                        help="A* 单任务最大迭代次数")
    args = parser.parse_args()

    print("=" * 60)
    print("  单机路径规划 (Single-UAV Planning)")
    print("  3.25 多维时空 A* + 3.26 多目标优化")
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

    # 2. 选择任务
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
    print(f"\n[2/5] 选中任务: {task.task_id} ({task.task_name})")
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

    # 3. 执行规划
    print("\n[3/5] 执行规划...")
    final_route, all_routes, warnings = plan_single_task(
        task, config, obstacle_set, perf,
        optimize=not args.no_optimize,
        max_iterations=args.max_iterations,
    )

    # 4. 保存结果
    print("\n[4/5] 保存结果...")
    result = PlanningResult(
        request_id=f"single-{task.task_id}",
        routes=all_routes,
        total_planning_time_ms=0,
        success_count=sum(1 for r in all_routes if r.status == "planned"),
        failed_count=sum(1 for r in all_routes if r.status != "planned"),
    )
    result_file = write_planning_result(result, args.output_dir)
    print(f"  结果: {result_file}")

    routes_dir = os.path.join(args.output_dir, "routes")
    ensure_output_dir(routes_dir)
    for r in all_routes:
        if r.status == "planned":
            write_route(r, routes_dir)

    # 航点指令
    if final_route.status == "planned" and not args.no_optimize:
        optimizer = RouteOptimizer(config, obstacle_set, perf)
        waypoints = optimizer.generate_waypoints(final_route.trajectory, min_spacing=80.0)
        wp_file = write_waypoints(waypoints, args.output_dir,
                                  f"waypoints_{task.task_id}.json")
        print(f"  航点: {wp_file} ({len(waypoints)} 个)")

    # 环境快照（供 visualize.py 使用）
    _save_environment(
        os.path.join(args.input_dir, "config.json"),
        os.path.join(args.input_dir, "static_obstacles.json"),
        os.path.join(args.input_dir, "dynamic_obstacles.json"),
        os.path.join(args.output_dir, "environment.json"),
    )

    # 5. 报告
    print("\n[5/5] 生成报告...")
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
    report_file = generate_report(result, config_info, obstacle_info, args.output_dir)
    print(f"  报告: {report_file}")

    print("\n" + "=" * 60)
    print(f"  完成! 规划状态: {'✓ 成功' if final_route.status == 'planned' else '✗ 失败'}")
    print(f"  输出: {os.path.abspath(args.output_dir)}")
    print(f"  可视化: python visualize.py --input {args.output_dir}")
    if warnings:
            print(f"  ⚠ {len(warnings)} 个兼容性警告，详见上文")
    print("=" * 60)


def _save_environment(config_path, static_path, dynamic_path, output_path):
    """保存环境快照"""
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