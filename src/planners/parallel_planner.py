"""
大规模并行规划引擎 (3.25 模块四 / 任务书 §三.2 Scheduler)

任务书 §三.2 Scheduler 职责：
- 任务拆分：接收多个规划任务并建立任务队列
- 任务优先级管理：根据任务优先级、紧急程度确定规划顺序
- 规划任务并行执行：调用多个 Planner 同时生成不同无人机航线
- 时空资源管理：维护已规划航线的空间和时间占用信息
- 冲突协调：发现航线冲突后触发重新规划、时间调整或高度调整
- 结果汇总：形成最终多机无冲突航线集合

任务书 §五.4 多机航线冲突避免：
"检测多个航线之间的空间和时间重叠。
 满足水平和垂直安全距离。
 必要时调整航点或时间窗口。"

本文件实现：
- ParallelPlanner: 多任务并行规划器
- SpatioTemporalOccupancyTable: 时空占用表（核心新模块）
- ConflictResolver: 冲突检测与重规划协调

性能优化（P2）：
- 共享一份 SpaceTimeGrid，避免每任务重建 30GB
- 支持 DISPATCH 模式：跳过 risk/heading，启用起点预校验
"""

from __future__ import annotations

import random
import time
import hashlib
import math
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Dict, List, Optional, Tuple

import numpy as np

from ..models.space import SpaceTimeConfig, SpaceTimeGrid
from ..models.obstacles import ObstacleSet
from ..models.aircraft import AircraftPerformance, FlightTask, Waypoint
from ..models.task import PlannedRoute, PlanningResult
from ..grid.cost import CostGrid, build_clearance_cost_grid
from .spacetime_astar import SpaceTimeAStar


# 占用表与冲突协调器独立成模块：SpaceTimeAStar 需要消费占用表来做
# 多机冲突【避免】，而本模块又依赖 A*，放在同一文件会循环导入。
# 对外仍从本模块 re-export，保持 `from ...parallel_planner import
# SpatioTemporalOccupancyTable` 这一既有写法可用。
from .occupancy import ConflictResolver, SpatioTemporalOccupancyTable

__all__ = ["ParallelPlanner", "SpatioTemporalOccupancyTable", "ConflictResolver"]


def _route_fingerprint(route: PlannedRoute) -> bytes:
    digest = hashlib.sha256(route.status.encode())
    points = np.asarray([(p.x, p.y, p.z, p.t) for p in route.trajectory], dtype=np.float64)
    digest.update(points.tobytes())
    return digest.digest()


class ParallelPlanner:
    """
    大规模并行规划引擎

    支持:
    - 多任务并行规划（线程/进程）
    - 优先级调度
    - 规划结果冲突检测
    - 性能统计与规模验证
    """

    def __init__(
        self,
        config: SpaceTimeConfig,
        obstacle_set: ObstacleSet,
        max_workers: int = 4,
        max_iterations: Optional[int] = None,
        use_cost_grid: bool = False,
        clearance: float = 200.0,
        clearance_cost: float = 3.0,
    ):
        self.config = config
        self.obstacle_set = obstacle_set
        self.max_workers = max_workers
        self.max_iterations = max_iterations

        # ★ 关键优化 (P2)：构建一份共享的时空网格，所有任务复用，
        # 消除原来每个任务各自重建 3MB 占用网格的浪费（10000 任务省 30GB 分配）。
        self._shared_grid = SpaceTimeGrid(config)
        if obstacle_set is not None:
            for obs in obstacle_set.static_obstacles:
                self._shared_grid.set_static_box(
                    x_min=obs.x - obs.width, x_max=obs.x + obs.width,
                    y_min=obs.y - obs.depth, y_max=obs.y + obs.depth,
                    z_min=obs.z, z_max=obs.z + obs.height,
                )
            for obs in obstacle_set.dynamic_obstacles:
                self._shared_grid.set_dynamic_obstacle(obs.trajectory, obs.safety_radius)

        # 通行代价层（与 occupancy 正交，只读共享，可跨线程复用）
        self._shared_cost_grid: Optional[CostGrid] = None
        if use_cost_grid:
            self._shared_cost_grid = build_clearance_cost_grid(
                self._shared_grid, obstacle_set,
                clearance=clearance, clearance_cost=clearance_cost,
            )

        # 共享的单进程规划器（用于线程模式和顺序回退）
        self._planner = SpaceTimeAStar(
            config=config,
            obstacle_set=obstacle_set,
            max_iterations=max_iterations,
            grid=self._shared_grid,
            cost_grid=self._shared_cost_grid,
        )

    def plan_batch(
        self,
        tasks: List[FlightTask],
        performances: Dict[str, AircraftPerformance],
        request_id: str = "batch-001",
        dispatch: bool = False,
        conflict_avoidance: bool = True,
        min_separation: float = 10.0,
        max_conflict_rounds: Optional[int] = None,
    ) -> PlanningResult:
        """
        批量并行规划

        Args:
            tasks: 飞行任务列表
            performances: 飞行器性能字典 {aircraft_id: AircraftPerformance}
            request_id: 请求 ID
            dispatch: 调度模式开关。True 时只跑 A* 不计算 risk/heading 等附加字段，
                      不改动任务起终点；占用或不可达时明确返回失败。
                      生产调度推荐 True；demo/报告用 False。
            conflict_avoidance: 是否做多机冲突避免（任务书 §五.4）。
                      默认开启：并行初规划 → 检测冲突 → 用占用表掩码重规划
                      受影响航线，直到无冲突。关闭后退化为"只规划不避让"，
                      各飞行器可能在同一格同一秒重叠（仅用于性能基准对比）。
            min_separation: 多机最小安全间隔（米）
            max_conflict_rounds: 冲突修复轮次预算；None 不限制次数

        Returns:
            PlanningResult: 批量规划结果
        """
        t0 = time.time()

        # 按优先级排序（高优先级先规划）
        sorted_tasks = sorted(tasks, key=lambda t: t.priority, reverse=True)

        routes: List[PlannedRoute] = []
        success_count = 0
        failed_count = 0

        # 多线程并行（共享内存，适合共享只读障碍物数据）
        routes = self._plan_multithread(sorted_tasks, performances, dispatch=dispatch)

        conflicts: List[dict]
        if conflict_avoidance:
            routes, conflicts, _replanned = self.avoid_conflicts(
                routes, sorted_tasks, performances,
                dispatch=dispatch, min_separation=min_separation,
                max_rounds=max_conflict_rounds,
            )
            # A returned trajectory is executable only after hard separation
            # conflicts are resolved. Keep the raw points and conflict records
            # for diagnosis, but exclude every affected route from successes.
            unresolved_route_ids = {
                conflict[key]
                for conflict in conflicts
                for key in ("route_a", "route_b")
                if key in conflict
            }
            for route in routes:
                if route.status == "planned" and route.route_id in unresolved_route_ids:
                    route.status = "failed"
                    reason = "存在未解决的多机安全间隔冲突，冲突协调结束后仍不可执行"
                    route.message = f"{route.message} | {reason}" if route.message else reason
        else:
            # 不避让也必须如实报告冲突，否则调用方会把"未检查"误当成"无冲突"
            conflicts = self.detect_conflicts_occupancy(
                routes, min_separation=min_separation
            )

        for route in routes:
            if route.status == "planned":
                success_count += 1
            else:
                failed_count += 1

        total_time = (time.time() - t0) * 1000

        result = PlanningResult(
            request_id=request_id,
            routes=routes,
            total_planning_time_ms=total_time,
            success_count=success_count,
            failed_count=failed_count,
            conflicts=conflicts,
        )

        return result

    def _plan_multithread(
        self,
        tasks: List[FlightTask],
        performances: Dict[str, AircraftPerformance],
        dispatch: bool = False,
    ) -> List[PlannedRoute]:
        """多线程并行规划"""
        routes = [None] * len(tasks)

        with ThreadPoolExecutor(max_workers=self.max_workers) as executor:
            future_to_idx = {}
            for idx, task in enumerate(tasks):
                perf = performances.get(task.aircraft_id, AircraftPerformance(aircraft_id=task.aircraft_id))
                future = executor.submit(
                    self._plan_one,
                    task,
                    perf,
                    idx,
                    dispatch,
                )
                future_to_idx[future] = idx

            for future in as_completed(future_to_idx):
                idx = future_to_idx[future]
                try:
                    routes[idx] = future.result()
                except MemoryError:
                    raise
                except Exception as e:
                    routes[idx] = PlannedRoute(
                        route_id=f"route-{idx:03d}",
                        task_id=tasks[idx].task_id,
                        aircraft_id=tasks[idx].aircraft_id,
                        status="failed",
                        message=f"规划异常: {str(e)}",
                        algorithm="ParallelPlanner",
                    )

        return [r for r in routes if r is not None]

    def _plan_one(
        self,
        task: FlightTask,
        performance: AircraftPerformance,
        idx: int,
        dispatch: bool = False,
        occupancy_mask: Optional[np.ndarray] = None,
        start_time: Optional[float] = None,
    ) -> PlannedRoute:
        """单任务规划（线程安全：共享只读 grid，dispatch 模式跳过附加计算）

        Args:
            occupancy_mask: 其它已规划航线的禁飞时空掩码（多机冲突避免）。
                None 表示不考虑其它飞行器，行为与单机规划一致。
            start_time: 覆盖起飞时刻（冲突协调的"延迟起飞"策略用）
        """
        # ★ 使用共享的 SpaceTimeGrid（不重建 3MB 数组）
        planner = SpaceTimeAStar(
            config=self.config,
            obstacle_set=self.obstacle_set,
            max_speed=performance.max_speed,
            max_iterations=self.max_iterations,
            grid=self._shared_grid,
            cost_grid=self._shared_cost_grid,
            min_altitude=performance.get_min_altitude(),
            max_altitude=performance.get_max_altitude(),
            max_climb_rate=performance.get_max_climb_rate(),
            max_descent_rate=performance.get_max_descent_rate(),
            occupancy_mask=occupancy_mask,
        )
        departure = task.departure_time if start_time is None else start_time

        start_wp = task.start
        end_wp = task.end

        # Dispatch changes reporting cost only; task coordinates stay exact.

        # 如果有多个途经点，分段规划
        if len(task.waypoints) > 2:
            return self._plan_multi_waypoint(task, performance, idx, planner, dispatch=dispatch,
                                              start_wp=start_wp, end_wp=end_wp,
                                              start_time=departure)

        route = planner.plan(
            start=(start_wp.x, start_wp.y, start_wp.z),
            goal=(end_wp.x, end_wp.y, end_wp.z),
            start_time=departure,
            task_id=task.task_id,
            aircraft_id=task.aircraft_id,
            route_id=f"route-{idx:03d}",
            extras=not dispatch,  # 调度模式跳过 risk/heading
        )
        return route

    def _snap_to_free(
        self, x: float, y: float, z: float, max_radius: int = 2,
    ) -> Tuple[float, float, float]:
        """把坐标微调到最近非障碍格（最多搜索 max_radius 圈）"""
        if not self.obstacle_set.is_static_blocked(x, y, z):
            return (x, y, z)
        ix, iy, iz = self.config.to_grid(x, y, z)
        for r in range(1, max_radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    for dz in range(-r, r + 1):
                        nix, niy, niz = ix + dx, iy + dy, iz + dz
                        if (
                            0 <= nix < self.config.nx
                            and 0 <= niy < self.config.ny
                            and 0 <= niz < self.config.nz
                        ):
                            wx, wy, wz = self.config.to_world(nix, niy, niz)
                            if not self.obstacle_set.is_static_blocked(wx, wy, wz):
                                return (wx, wy, wz)
        return (x, y, z)  # 找不到空闲格，保持原坐标

    def _plan_multi_waypoint(
        self,
        task: FlightTask,
        performance: AircraftPerformance,
        idx: int,
        planner: SpaceTimeAStar,
        dispatch: bool = False,
        start_wp=None,
        end_wp=None,
        start_time: Optional[float] = None,
    ) -> PlannedRoute:
        """多航点分段规划"""
        from ..models.task import TrajectoryPoint

        if start_wp is None:
            start_wp = task.start
        if end_wp is None:
            end_wp = task.end

        all_trajectory = []
        current_time = task.departure_time if start_time is None else start_time
        total_planning_time = 0.0

        # Preserve every user-specified waypoint, including dispatch mode.
        waypoints = list(task.waypoints)

        for i in range(len(waypoints) - 1):
            wp_start = waypoints[i]
            wp_end = waypoints[i + 1]

            seg_route = planner.plan(
                start=(wp_start.x, wp_start.y, wp_start.z),
                goal=(wp_end.x, wp_end.y, wp_end.z),
                start_time=current_time,
                task_id=task.task_id,
                aircraft_id=task.aircraft_id,
                route_id=f"route-{idx:03d}-seg{i}",
                extras=not dispatch,
            )

            total_planning_time += seg_route.planning_time_ms

            if seg_route.status != "planned":
                return PlannedRoute(
                    route_id=f"route-{idx:03d}",
                    task_id=task.task_id,
                    aircraft_id=task.aircraft_id,
                    status="failed",
                    message=f"航段 {i} 规划失败: {seg_route.message}",
                    algorithm="SpaceTimeAStar-MultiWaypoint",
                    planning_time_ms=total_planning_time,
                )

            # 拼接轨迹（跳过重复的）
            if all_trajectory:
                all_trajectory.extend(seg_route.trajectory[1:])
            else:
                all_trajectory.extend(seg_route.trajectory)

            current_time = seg_route.trajectory[-1].t + wp_end.hover_time

        route = PlannedRoute(
            route_id=f"route-{idx:03d}",
            task_id=task.task_id,
            aircraft_id=task.aircraft_id,
            trajectory=all_trajectory,
            status="planned",
            algorithm="SpaceTimeAStar-MultiWaypoint",
            planning_time_ms=total_planning_time,
        )
        # dispatch 模式下只算 distance/time（cheaper），full 模式算 energy 等
        if dispatch:
            route.compute_metrics()
        else:
            route.compute_metrics(performance)
            # 分段时每个 seg_route 各自算过 risk，但拼接后的整条航路是新的
            # PlannedRoute 对象，必须在这里对合并轨迹重算一次，否则
            # min_obstacle_distance / max_risk / avg_risk 会停留在默认值 0.0
            planner._compute_risk(route)
        return route

    def detect_conflicts(self, routes: List[PlannedRoute], min_separation: float = 10.0) -> List[dict]:
        """
        检测多条航线之间的冲突（飞行器间碰撞风险）

        Args:
            routes: 规划航线列表
            min_separation: 最小安全间隔 (米)

        Returns:
            冲突列表 [{route_a, route_b, time, position, distance}, ...]
        """
        conflicts = []
        for i in range(len(routes)):
            for j in range(i + 1, len(routes)):
                if routes[i].status != "planned" or routes[j].status != "planned":
                    continue
                conflict = self._check_pair_conflict(routes[i], routes[j], min_separation)
                if conflict:
                    conflicts.extend(conflict)
        return conflicts

    def _check_pair_conflict(
        self, route_a: PlannedRoute, route_b: PlannedRoute, min_separation: float
    ) -> Optional[List[dict]]:
        """检查两条航线之间的冲突"""
        conflicts = []
        traj_a = route_a.trajectory
        traj_b = route_b.trajectory

        # 时间对齐检查
        for pa in traj_a:
            for pb in traj_b:
                if abs(pa.t - pb.t) > 1.0:  # 时间差超过1秒不检查
                    continue
                dx = pa.x - pb.x
                dy = pa.y - pb.y
                dz = pa.z - pb.z
                dist = np.sqrt(dx * dx + dy * dy + dz * dz)
                if dist < min_separation:
                    conflicts.append({
                        "route_a": route_a.route_id,
                        "route_b": route_b.route_id,
                        "time": round(pa.t, 2),
                        "position_a": (round(pa.x, 1), round(pa.y, 1), round(pa.z, 1)),
                        "position_b": (round(pb.x, 1), round(pb.y, 1), round(pb.z, 1)),
                        "distance": round(dist, 2),
                    })
        return conflicts if conflicts else None

    def build_occupancy_table(
        self,
        routes: List[PlannedRoute],
        min_separation: float = 10.0,
        safety_radius: float = 0.0,
    ) -> SpatioTemporalOccupancyTable:
        """
        为一组航线构建时空占用表（任务书 §三.2 时空资源管理）

        Args:
            routes: 航线列表（自动过滤 status != "planned"）
            min_separation: 多机最小安全间隔（米）
            safety_radius: 每条航线的占用半径（米）
        """
        table = SpatioTemporalOccupancyTable(self.config, min_separation=min_separation)
        for route in routes:
            if route.status == "planned":
                table.mark_route(route, safety_radius=safety_radius)
        return table

    def detect_conflicts_occupancy(
        self,
        routes: List[PlannedRoute],
        min_separation: float = 10.0,
        time_window: float = 5.0,
        safety_radius: float = 0.0,
        max_routes: Optional[int] = None,
    ) -> List[dict]:
        """
        基于时空占用表的冲突检测（任务书 §五.4）

        与 :meth:`detect_conflicts` 的 O(n²·m²) 两两比对不同，这里按航线
        顺序增量登记：查冲突 → 登记占用，每条航线只与"已登记的"航线比较，
        避免重复比对同一对航线。冲突条目去重后返回。

        Args:
            routes: 航线列表（自动过滤 status != "planned"）
            min_separation: 最小安全间隔（米）
            time_window: 时间对齐窗口（秒）
            safety_radius: 占用半径（米）
            max_routes: 最多检查多少条航线（超大规模时限制开销）。
                超限时**随机抽样**而不是取前 N 条——前缀切片会系统性地
                只检查高优先级/靠前的任务，抽样才是无偏的估计。
                注意：抽样只覆盖"抽中的航线两两之间"的冲突，
                与未抽中航线的冲突不会被发现，因此该参数仅用于
                估算冲突规模，**不能**用于安全放行判定。

        Returns:
            去重后的冲突列表
        """
        planned = [r for r in routes if r.status == "planned"]
        if max_routes is not None and len(planned) > max_routes:
            planned = random.Random(0).sample(planned, max_routes)

        table = SpatioTemporalOccupancyTable(self.config, min_separation=min_separation)
        seen = set()
        conflicts = []
        for route in planned:
            for c in table.find_conflicts(route, min_separation, time_window):
                key = (c["route_a"], c["route_b"], c["time"])
                if key in seen:
                    continue
                seen.add(key)
                conflicts.append(c)
            table.mark_route(route, safety_radius=safety_radius)
        return conflicts

    def _route_index(self, route_id: str) -> Optional[int]:
        """route-{idx:03d} → idx（即 sorted_tasks 中的下标）"""
        try:
            return int(route_id.split("-")[1])
        except (IndexError, ValueError):
            return None

    def avoid_conflicts(
        self,
        routes: List[PlannedRoute],
        sorted_tasks: List[FlightTask],
        performances: Dict[str, AircraftPerformance],
        dispatch: bool = False,
        min_separation: float = 10.0,
        max_rounds: Optional[int] = None,
        max_delay_steps: Optional[int] = None,
        time_window: float = 5.0,
    ) -> Tuple[List[PlannedRoute], List[dict], int]:
        """
        多机航线冲突【避免】（任务书 §五.4 / §三.2 冲突协调）

        这是把 `SpatioTemporalOccupancyTable` 真正接进规划闭环的一步——
        没有它，各飞行器只是"各自规划完再事后比对"，结果会出现两机
        在同一格同一秒重叠（实测 0.0m）却被标记为 planned。

        流程（任务书 §三.2「发现航线冲突后触发重新规划」）：
          1. 用全部已规划航线建时空占用表
          2. 检测冲突；对每对冲突，选**优先级较低**的一方
          3. 把占用表转成稠密禁飞掩码（排除该航线自身），喂给 A* 重规划。
             A* 自带等待动作，因此"时间调整"（延迟出发）与"空间绕行"
             由搜索自动二选一，无需人工分支
          4. 若起点本身被占（搜索第一步就出不去），按 dt 逐格延迟重试——
             这是任务书 §三.2 的"时间调整"策略
          5. 重复修复，直到无冲突、状态重复或无法改进

        每轮内**串行**重规划：新掩码必须包含本轮刚更新的航线，
        否则两条同时重规划的航线可能撞在一起。

        Args:
            routes: plan_batch 产出的航线（与 sorted_tasks 下标一一对应）
            sorted_tasks: 按优先级降序排列的任务
            performances: 飞行器性能字典
            dispatch: 调度模式
            min_separation: 多机最小安全间隔（米）
            max_rounds: 修复轮次预算；None 不限制次数
            max_delay_steps: 延迟尝试预算；None 搜索到场景时域末端
            time_window: 冲突检测的时间对齐窗口（秒）

        Returns:
            (new_routes, remaining_conflicts, replanned_count)
        """
        table = SpatioTemporalOccupancyTable(self.config, min_separation=min_separation)
        for r in routes:
            if r.status == "planned":
                table.mark_route(r)

        current = list(routes)
        replanned = 0

        seen_states = set()
        rounds = 0
        while max_rounds is None or rounds < max_rounds:
            state = tuple(_route_fingerprint(route) for route in current)
            if state in seen_states:
                break
            seen_states.add(state)
            rounds += 1
            conflicts = self.detect_conflicts_occupancy(
                current, min_separation=min_separation, time_window=time_window
            )
            if not conflicts:
                return current, [], replanned

            # 每对冲突选优先级低的一方重规划（sorted_tasks 已按优先级降序，
            # 因此下标越大优先级越低）
            targets: set = set()
            for c in conflicts:
                ia = self._route_index(c.get("route_a", ""))
                ib = self._route_index(c.get("route_b", ""))
                if ia is None or ib is None:
                    continue
                if max(ia, ib) < len(sorted_tasks):
                    targets.add(max(ia, ib))
            if not targets:
                break

            improved = False
            for idx in sorted(targets):
                if idx >= len(current):
                    continue
                old = current[idx]
                if old.status != "planned":
                    continue
                new = self._replan_avoiding(
                    task=sorted_tasks[idx],
                    performance=performances.get(
                        sorted_tasks[idx].aircraft_id,
                        AircraftPerformance(aircraft_id=sorted_tasks[idx].aircraft_id),
                    ),
                    idx=idx,
                    dispatch=dispatch,
                    table=table,
                    old_route=old,
                    max_delay_steps=max_delay_steps,
                )
                if new.status != "planned" or _route_fingerprint(new) == _route_fingerprint(old):
                    continue
                table.unmark_route(old)
                table.mark_route(new)
                current[idx] = new
                improved = True
                replanned += 1

            if not improved:
                break  # 一轮都改不动，继续跑也是白跑

        remaining = self.detect_conflicts_occupancy(
            current, min_separation=min_separation, time_window=time_window
        )
        return current, remaining, replanned

    def _replan_avoiding(
        self,
        task: FlightTask,
        performance: AircraftPerformance,
        idx: int,
        dispatch: bool,
        table: SpatioTemporalOccupancyTable,
        old_route: Optional[PlannedRoute] = None,
        max_delay_steps: Optional[int] = None,
    ) -> PlannedRoute:
        """带禁飞掩码重规划一条航线；起点被占则逐格延迟重试

        Args:
            old_route: 占用表里登记的**同一条**旧航线。掩码要把它自己的
                占用排除掉，否则起点永远被自己挡住。这里必须传航线对象
                （而不是 route_id）——排除要按它自己的轨迹逐格修正，
                是 O(|该航线|)，而不是按 id 扫全表 O(|occupancy|)。
        """
        mask = table.to_dense_mask(exclude_route=old_route)
        route = self._plan_one(
            task, performance, idx, dispatch, occupancy_mask=mask
        )
        if route.status == "planned":
            return route

        # 起点/终点在其它飞行器占用格里 → 时间调整（延迟起飞）
        available_steps = max(0, math.ceil((self.config.t_max - task.departure_time) / self.config.dt) - 1)
        if max_delay_steps is None:
            max_delay_steps = available_steps
        else:
            max_delay_steps = min(max_delay_steps, available_steps)
        for k in range(1, max_delay_steps + 1):
            route = self._plan_one(
                task, performance, idx, dispatch,
                occupancy_mask=mask,
                start_time=task.departure_time + k * self.config.dt,
            )
            if route.status == "planned":
                return route
        return route

    def resolve_conflicts(
        self,
        conflicts: List[dict],
        routes: List[PlannedRoute],
        max_delay_s: float = 60.0,
    ) -> List[dict]:
        """
        冲突协调 (任务书 §三.2 冲突协调)

        为每条冲突给出协调建议：
        - 冲突时刻早于可接受延迟窗口 → 建议"延迟起飞"（等对方先飞过）
        - 冲突时刻过晚，延迟反而会撞进冲突窗口 → 建议"高度调整"（上移一个网格层）

        注意 ConflictResolver.suggest_delay 会把结果截断到 max_delay_s，
        因此这里不能用"延迟量是否超限"来判断分支（那样 altitude 分支永远不可达），
        而是直接用冲突时刻与 max_delay_s 的关系判断。

        Args:
            conflicts: detect_conflicts_occupancy 的输出
            routes: 航线列表（用于取当前高度）
            max_delay_s: 可接受的最大延迟（秒）

        Returns:
            带 resolution 字段的冲突列表
        """
        resolver = ConflictResolver(self.config, min_separation=10.0)
        z_by_route = {
            r.route_id: (r.trajectory[len(r.trajectory) // 2].z if r.trajectory else 0.0)
            for r in routes
        }
        route_by_id = {r.route_id: r for r in routes}
        resolved = []
        for c in conflicts:
            item = dict(c)
            if c.get("time", 0.0) <= max_delay_s:
                item["resolution"] = "delay"
                # 传入对方航线 → 按对方实际通过冲突点的时间区间计算延迟量
                item["suggested_delay_s"] = round(
                    resolver.suggest_delay(
                        c, max_delay_s, other_route=route_by_id.get(c.get("route_b", ""))
                    ) or 0.0,
                    1,
                )
            else:
                current_z = z_by_route.get(c["route_a"], 0.0)
                item["resolution"] = "altitude_shift"
                item["suggested_z"] = round(
                    resolver.suggest_altitude_shift(current_z), 1
                )
            resolved.append(item)
        return resolved

    def scale_test(self, num_tasks: int = 100) -> dict:
        """
        规模验证：测试大规模任务下的规划性能

        Returns:
            性能统计字典
        """
        import random

        tasks = []
        performances = {}
        for i in range(num_tasks):
            from ..models.aircraft import Waypoint, FlightTask, AircraftPerformance

            aircraft_id = f"uav-{i:03d}"
            perf = AircraftPerformance(aircraft_id=aircraft_id)
            performances[aircraft_id] = perf

            start_x = random.uniform(100, self.config.x_max - 100)
            start_y = random.uniform(100, self.config.y_max - 100)
            end_x = random.uniform(100, self.config.x_max - 100)
            end_y = random.uniform(100, self.config.y_max - 100)

            task = FlightTask(
                task_id=f"task-{i:03d}",
                task_name=f"scale-test-{i}",
                aircraft_id=aircraft_id,
                waypoints=[
                    Waypoint(wp_id="start", x=start_x, y=start_y, z=50.0, wp_type="start"),
                    Waypoint(wp_id="end", x=end_x, y=end_y, z=50.0, wp_type="end"),
                ],
                priority=random.randint(1, 10),
            )
            tasks.append(task)

        t0 = time.time()
        result = self.plan_batch(tasks, performances, request_id=f"scale-{num_tasks}")
        total_time = (time.time() - t0) * 1000

        return {
            "num_tasks": num_tasks,
            "total_time_ms": round(total_time, 2),
            "avg_time_per_task_ms": round(total_time / num_tasks, 2),
            "success_rate": round(result.success_count / num_tasks * 100, 1),
            "success_count": result.success_count,
            "failed_count": result.failed_count,
        }
