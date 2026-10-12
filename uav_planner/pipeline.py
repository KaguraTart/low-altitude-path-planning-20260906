"""
规划编排流水线

本模块把算法内核（``src/``）的各个组件串成两条完整流程：

- :func:`run_single_task` —— 单任务：时空 A* → 多目标优化
- :func:`run_batch`       —— 多任务：并行规划 → 多机冲突避免
- :func:`validate_tasks_with_aircraft` —— 前置的飞行器需求兼容性校验

**为什么放在 SDK 里而不是命令行脚本里**：这些编排逻辑原先定义在
``plan_single.py`` / ``plan_multi.py`` 两个顶层脚本中，SDK 反过来 import 它们。
这让依赖方向倒挂了——交付「仅接口层」时（不含命令行脚本）SDK 会直接
import 失败。现在把编排下沉到 SDK，命令行脚本改为调用本模块：

    tools/plan_single.py ──┐
    tools/plan_multi.py  ──┼──→ uav_planner/pipeline.py ──→ src/
    纯接口调用方          ──┘

依赖方向单一，接口层可以独立交付。

这些函数直接复用内核模块，不含任何命令行参数解析，因此在 SDK、HTTP 服务、
命令行三种场景下行为完全一致。
"""

from __future__ import annotations

import time
from typing import Any, Dict, List, Optional, Tuple

from src.models.aircraft import AircraftPerformance, FlightTask
from src.models.obstacles import ObstacleSet
from src.models.space import SpaceTimeConfig
from src.models.task import PlannedRoute, PlanningResult
from src.planners.parallel_planner import ParallelPlanner
from src.planners.route_optimizer import RouteOptimizer
from src.planners.spacetime_astar import SpaceTimeAStar


def validate_tasks_with_aircraft(
    tasks: List[FlightTask],
    performances: Dict[str, AircraftPerformance],
) -> Tuple[List[FlightTask], List[Tuple[str, List[str]]]]:
    """
    校验每个任务的飞行器需求兼容性（载荷 / 续航 / 速度 / 升限）

    Returns:
        (compatible_tasks, incompatible_pairs)
        - compatible_tasks: 通过校验的任务列表
        - incompatible_pairs: ``[(task_id, [问题]), ...]``
    """
    compatible: List[FlightTask] = []
    incompatible: List[Tuple[str, List[str]]] = []
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


def run_single_task(
    task: FlightTask,
    config: SpaceTimeConfig,
    obstacle_set: ObstacleSet,
    perf: AircraftPerformance,
    optimize: bool = True,
    max_iterations: int = 2_000_000,
    cost_grid: Optional[Any] = None,
    verbose: bool = True,
) -> Tuple[PlannedRoute, List[PlannedRoute], List[str]]:
    """
    对单个任务执行完整规划：时空 A* → 多目标优化

    多航点任务（``len(task.waypoints) > 2``）会**分段规划**后拼接，
    与只给起终点的结果不同——中间经停点不会被丢掉。

    Args:
        verbose: 是否打印进度。SDK / HTTP 调用传 ``False``，避免污染服务日志
                 与被调用方的 stdout；命令行传 ``True`` 保持交互输出。

    Returns:
        (final_route, all_routes, warnings)
        - final_route: 优化后的最终航线
        - all_routes: A* 与优化产生的所有中间结果
        - warnings: 飞行器与任务需求不匹配等非致命告警
    """
    warnings: List[str] = []

    def _log(msg: str = ""):
        if verbose:
            print(msg)

    # 1. 任务 + 飞行器兼容性校验
    valid, msg = task.validate()
    if not valid:
        raise ValueError(f"任务定义非法: {msg}")
    compatible, problems = task.check_aircraft_compatibility(perf)
    if not compatible:
        warnings.extend(problems)
        _log(f"[!] 飞行器 {perf.aircraft_id} 与任务 {task.task_id} 不兼容:")
        for p in problems:
            _log(f"    - {p}")

    # 2. A* 规划（包线约束一并下传：高度上下限 + 升降率）
    _log(f"\n[1/2] 时空 A* 规划 ({task.task_id})...")
    planner = SpaceTimeAStar(
        config=config,
        obstacle_set=obstacle_set,
        max_speed=perf.get_max_speed(),
        max_iterations=max_iterations,
        cost_grid=cost_grid,
        min_altitude=perf.get_min_altitude(),
        max_altitude=perf.get_max_altitude(),
        max_climb_rate=perf.get_max_climb_rate(),
        max_descent_rate=perf.get_max_descent_rate(),
    )

    all_routes: List[PlannedRoute] = []
    t0 = time.time()

    if len(task.waypoints) > 2:
        # 多航点分段规划
        traj: List[Any] = []
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
                _log(f"    航段 {seg_idx} 失败: {seg_route.message}")
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
            _log(f"    成功: {route.total_distance:.1f}m, {route.total_time:.1f}s, "
                 f"{len(traj)}点")
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
            _log(f"    成功: {route.total_distance:.1f}m, {route.total_time:.1f}s")
        else:
            _log(f"    失败: {route.message}")

    final_route = all_routes[-1]

    # 3. 多目标优化
    if optimize and final_route.status == "planned":
        _log(f"\n[2/2] 多目标航线优化 ({task.task_id})...")
        optimizer = RouteOptimizer(config, obstacle_set, perf, cost_grid=cost_grid)
        optimized = optimizer.optimize(task, route_id=f"opt-{task.task_id}")
        all_routes.append(optimized)
        final_route = optimized
        _log(f"    优化完成: {final_route.total_distance:.1f}m, "
             f"{final_route.total_time:.1f}s")
        _log(f"      风险: max={final_route.max_risk:.4f}, "
             f"avg={final_route.avg_risk:.4f}, "
             f"最近障碍={final_route.min_obstacle_distance:.1f}m")
    else:
        _log("\n[2/2] 跳过优化")

    _log(f"\n  规划耗时: {(time.time()-t0)*1000:.1f} ms")
    return final_route, all_routes, warnings


def run_batch(
    tasks: List[FlightTask],
    performances: Dict[str, AircraftPerformance],
    config: SpaceTimeConfig,
    obstacle_set: ObstacleSet,
    dispatch: bool = False,
    workers: int = 4,
    max_iterations: int = 200_000,
    use_cost_grid: bool = False,
    conflict_avoidance: bool = True,
    min_separation: float = 10.0,
    clearance: float = 200.0,
    clearance_cost: float = 3.0,
    request_id: Optional[str] = None,
) -> Tuple[PlanningResult, ParallelPlanner]:
    """
    多任务批量并行规划，返回 ``(结果, 规划器实例)``

    规划器实例保留下来，供后续冲突检测复用其共享占用网格，避免重复构建
    3MB 网格。

    ``conflict_avoidance=True``（默认）时，规划结果已经过多机冲突避免；
    关闭后 ``result.conflicts`` **仍会如实报告**检测到的冲突，不会把
    「没检查」包装成「无冲突」。
    """
    planner = ParallelPlanner(
        config=config, obstacle_set=obstacle_set,
        max_workers=workers,
        max_iterations=max_iterations,
        use_cost_grid=use_cost_grid,
        clearance=clearance,
        clearance_cost=clearance_cost,
    )
    result = planner.plan_batch(
        tasks, performances,
        request_id=request_id or f"multi-{len(tasks)}",
        dispatch=dispatch,
        conflict_avoidance=conflict_avoidance,
        min_separation=min_separation,
    )
    return result, planner
