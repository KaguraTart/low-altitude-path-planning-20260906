#!/usr/bin/env python3
"""
规划运行脚本：低空无人机路径规划调度（仅规划，不做可视化）

功能:
1. 加载配置、障碍物、飞行器和任务数据
2. 执行多维时空 A* 路径规划 (3.25)
3. 执行多目标航线优化 (3.26)
4. 写出 JSON / Markdown 报告到 --output-dir

可视化由独立脚本 visualize.py 负责：
    python visualize.py --input output/demo/

用法:
    python run_planning.py
    python run_planning.py --data-dir ./data --output-dir ./output/demo
    python run_planning.py --mode spacetime
    python run_planning.py --mode optimize
    python run_planning.py --mode batch
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
from src.planners.parallel_planner import ParallelPlanner
from src.planners.route_optimizer import RouteOptimizer


def main():
    parser = argparse.ArgumentParser(description="低空无人机路径规划调度（仅规划）")
    parser.add_argument("--data-dir", default="./data", help="输入数据目录")
    parser.add_argument(
        "--output-dir", default="./output/demo",
        help="输出目录（JSON/MD 结果；可视化由 visualize.py 另存）",
    )
    parser.add_argument(
        "--mode", choices=["spacetime", "optimize", "both", "batch"], default="both",
        help="规划模式: spacetime=仅A*, optimize=仅优化, both=A*+优化, batch=批量并行",
    )
    parser.add_argument(
        "--workers", type=int, default=4,
        help="batch 模式下的并行线程数",
    )
    parser.add_argument(
        "--max-iterations", type=int, default=2_000_000,
        help="A* 单任务最大迭代次数",
    )
    args = parser.parse_args()

    data_dir = args.data_dir
    output_dir = args.output_dir
    ensure_output_dir(output_dir)

    print("=" * 60)
    print("  低空无人机路径规划调度系统 [规划模式]")
    print("  3.25 多维时空大规模并行规划算法")
    print("  3.26 智能飞行线路规划算法")
    print("=" * 60)

    # 1. 加载数据
    print("\n[1/5] 加载输入数据...")
    config = load_config(os.path.join(data_dir, "config.json"))
    print(f"  时空网格: {config.nx}x{config.ny}x{config.nz} (空间), {config.nt} 时间步")

    obstacle_set = load_obstacle_set(
        os.path.join(data_dir, "static_obstacles.json"),
        os.path.join(data_dir, "dynamic_obstacles.json"),
    )
    print(f"  静态障碍物: {len(obstacle_set.static_obstacles)} 个")
    print(f"  动态障碍物: {len(obstacle_set.dynamic_obstacles)} 个")

    performances = load_aircraft_performances(os.path.join(data_dir, "aircraft_config.json"))
    print(f"  飞行器配置: {len(performances)} 架")

    tasks = load_tasks(os.path.join(data_dir, "tasks.json"))
    print(f"  飞行任务: {len(tasks)} 个")

    # 2. 执行规划
    print(f"\n[2/5] 执行路径规划 (模式: {args.mode})...")
    t_start = time.time()

    all_routes = []

    if args.mode in ("spacetime", "both"):
        print("\n  ---  3.25 多维时空 A* 规划 ---")
        for i, task in enumerate(tasks):
            perf = performances.get(task.aircraft_id, AircraftPerformance(aircraft_id=task.aircraft_id))
            planner = SpaceTimeAStar(
                config=config,
                obstacle_set=obstacle_set,
                max_speed=perf.max_speed,
                max_iterations=args.max_iterations,
            )

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
                        print(f"    [{task.task_id}] 航段 {seg_idx} 失败: {seg_route.message}")
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
                    print(f"    [{task.task_id}] 成功: {route.total_distance:.1f}m, "
                          f"{route.total_time:.1f}s, {len(traj)}点")
                else:
                    all_routes.append(PlannedRoute(
                        route_id=f"astar-{task.task_id}",
                        task_id=task.task_id,
                        aircraft_id=task.aircraft_id,
                        status="failed",
                        message="多航点规划失败",
                    ))
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
                    print(f"    [{task.task_id}] 成功: {route.total_distance:.1f}m, "
                          f"{route.total_time:.1f}s")
                else:
                    print(f"    [{task.task_id}] 失败: {route.message}")

    if args.mode in ("optimize", "both"):
        print("\n  ---  3.26 多目标航线优化 ---")
        for i, task in enumerate(tasks):
            perf = performances.get(task.aircraft_id, AircraftPerformance(aircraft_id=task.aircraft_id))
            optimizer = RouteOptimizer(config, obstacle_set, perf)
            route = optimizer.optimize(task, route_id=f"opt-{task.task_id}")
            all_routes.append(route)

            if route.status == "planned":
                print(f"    [{task.task_id}] 优化完成: {route.total_distance:.1f}m, "
                      f"{route.total_time:.1f}s")
                print(f"      风险: max={route.max_risk:.4f}, avg={route.avg_risk:.4f}, "
                      f"最近障碍={route.min_obstacle_distance:.1f}m")
                # 生成航点
                waypoints = optimizer.generate_waypoints(route.trajectory, min_spacing=80.0)
                wp_file = write_waypoints(waypoints, output_dir, f"waypoints_{task.task_id}.json")
                print(f"      航点文件: {wp_file} ({len(waypoints)} 个航点)")
            else:
                print(f"    [{task.task_id}] 优化失败: {route.message}")

    if args.mode == "batch":
        print("\n  ---  大规模并行规划 ---")
        planner = ParallelPlanner(
            config, obstacle_set, max_workers=args.workers,
            max_iterations=args.max_iterations,
        )
        result = planner.plan_batch(tasks, performances, request_id="batch-demo")
        all_routes.extend(result.routes)
        print(f"    批量规划: {result.success_count}成功 / "
              f"{result.failed_count}失败, 耗时 {result.total_planning_time_ms:.1f}ms")

    planning_time = (time.time() - t_start) * 1000
    print(f"\n  规划总耗时: {planning_time:.1f} ms")

    # 3. 保存规划结果（不含可视化）
    print("\n[3/5] 保存规划结果...")
    result = PlanningResult(
        request_id=f"demo-{args.mode}",
        routes=all_routes,
        total_planning_time_ms=planning_time,
        success_count=sum(1 for r in all_routes if r.status == "planned"),
        failed_count=sum(1 for r in all_routes if r.status != "planned"),
    )
    result_file = write_planning_result(result, output_dir)
    print(f"  结果文件: {result_file}")

    # 同步保存环境快照（供 visualize.py 使用）
    _save_environment(os.path.join(data_dir, "config.json"),
                      os.path.join(data_dir, "static_obstacles.json"),
                      os.path.join(data_dir, "dynamic_obstacles.json"),
                      os.path.join(output_dir, "environment.json"))
    print(f"  环境快照: {os.path.join(output_dir, 'environment.json')}")

    routes_dir = os.path.join(output_dir, "routes")
    ensure_output_dir(routes_dir)
    for route in all_routes:
        if route.status == "planned":
            write_route(route, routes_dir)
    print(f"  单航线: {routes_dir}/*.json")

    # 4. 生成 Markdown 报告（不含可视化）
    print("\n[4/5] 生成规划报告...")
    config_info = {
        "空间范围": f"{config.x_min}-{config.x_max} x {config.y_min}-{config.y_max} x {config.z_min}-{config.z_max} m",
        "空间分辨率": f"{config.dx} x {config.dy} x {config.dz} m",
        "网格维度": f"{config.nx} x {config.ny} x {config.nz}",
        "时间范围": f"{config.t_min}-{config.t_max} s",
        "时间分辨率": f"{config.dt} s",
        "时间步数": f"{config.nt}",
    }
    obstacle_info = obstacle_set.summary()
    report_file = generate_report(result, config_info, obstacle_info, output_dir)
    print(f"  规划报告: {report_file}")

    # 5. 提示后续可视化步骤
    print("\n[5/5] 规划完成 ✓ (可视化请运行 visualize.py)")
    print(f"  输出目录: {os.path.abspath(output_dir)}")
    print(f"  可视化:   python visualize.py --input {output_dir}")
    print("=" * 60)


def _save_environment(config_path: str, static_path: str, dynamic_path: str, output_path: str):
    """把规划环境的 JSON 快照到输出目录（供 visualize.py 复现障碍物）"""
    snap = {}
    for label, path in [("config", config_path), ("static_obstacles", static_path), ("dynamic_obstacles", dynamic_path)]:
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as f:
                snap[label] = json.load(f)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(snap, f, ensure_ascii=False, indent=2)


if __name__ == "__main__":
    main()