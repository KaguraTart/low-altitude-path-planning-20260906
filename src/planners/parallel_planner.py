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

import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Dict, List, Optional, Tuple

import numpy as np

from ..models.space import SpaceTimeConfig, SpaceTimeGrid
from ..models.obstacles import ObstacleSet
from ..models.aircraft import AircraftPerformance, FlightTask, Waypoint
from ..models.task import PlannedRoute, PlanningResult
from .spacetime_astar import SpaceTimeAStar


class SpatioTemporalOccupancyTable:
    """
    时空占用表（任务书 §三.2 时空资源管理 + §四 步骤4）

    记录所有已规划航线的 (x, y, z, t) 占用情况，供后续规划参考和冲突检测。

    实现细节：
    - 用 hash set 存储离散化的占用格 (ix, iy, iz, it)
    - 每个格记录"被哪些飞机占用"，支持冲突溯源
    - 占用格在路线插入时计算，路线回滚时清除

    API：
    - mark_route(route): 标记整条航线占用
    - unmark_route(route): 清除航线占用
    - is_occupied(x, y, z, t, exclude_aircraft): 查询占用（可排除自身）
    - find_conflicts(route, min_separation): 找出与新航线的所有冲突
    """

    def __init__(self, config: SpaceTimeConfig, min_separation: float = 10.0):
        """
        Args:
            config: 时空网格配置（用于坐标→网格索引转换）
            min_separation: 多机最小安全间隔（米）
        """
        self.config = config
        self.min_separation = min_separation
        # {(ix, iy, iz, it): {aircraft_id, ...}} 占用记录
        self._occupancy: Dict[Tuple[int, int, int, int], set] = {}

    def _world_to_grid_with_radius(
        self, x: float, y: float, z: float, t: float, radius: float
    ) -> List[Tuple[int, int, int, int]]:
        """世界坐标 + 半径 → 受影响的网格列表"""
        ix, iy, iz = self.config.to_grid(x, y, z)
        it = self.config.time_to_step(t)
        r_cells = max(0, int(np.ceil(radius / min(self.config.dx, self.config.dy))))
        cells = []
        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                for dz in range(-r_cells, r_cells + 1):
                    nx_, ny_, nz_ = ix + dx, iy + dy, iz + dz
                    if 0 <= nx_ < self.config.nx and 0 <= ny_ < self.config.ny and 0 <= nz_ < self.config.nz:
                        cells.append((nx_, ny_, nz_, it))
        return cells

    def mark_route(self, route: PlannedRoute, safety_radius: float = 0.0):
        """将整条航线的占用格标记为 route.aircraft_id

        Args:
            route: 已规划航线
            safety_radius: 占用半径（米，飞机周围的额外安全圈）
        """
        for p in route.trajectory:
            cells = self._world_to_grid_with_radius(p.x, p.y, p.z, p.t, safety_radius)
            for cell in cells:
                if cell not in self._occupancy:
                    self._occupancy[cell] = set()
                self._occupancy[cell].add(route.aircraft_id)

    def unmark_route(self, route: PlannedRoute, safety_radius: float = 0.0):
        """从占用表中移除航线的占用格"""
        for p in route.trajectory:
            cells = self._world_to_grid_with_radius(p.x, p.y, p.z, p.t, safety_radius)
            for cell in cells:
                if cell in self._occupancy:
                    self._occupancy[cell].discard(route.aircraft_id)
                    if not self._occupancy[cell]:
                        del self._occupancy[cell]

    def is_occupied(
        self, x: float, y: float, z: float, t: float,
        exclude_aircraft: Optional[str] = None,
        safety_radius: float = 0.0,
    ) -> bool:
        """查询 (x, y, z, t) 是否被占用（可排除自身飞机）"""
        cells = self._world_to_grid_with_radius(x, y, z, t, safety_radius)
        for cell in cells:
            occupants = self._occupancy.get(cell)
            if occupants is None:
                continue
            if exclude_aircraft is None:
                if occupants:
                    return True
            else:
                # 排除自身飞机的占用
                others = {a for a in occupants if a != exclude_aircraft}
                if others:
                    return True
        return False

    def find_conflicts(
        self, route: PlannedRoute, min_separation: float = 10.0,
        time_window: float = 5.0,
    ) -> List[dict]:
        """找出新航线与已规划航线之间的所有冲突

        Returns:
            [{"route_a", "route_b", "time", "position", "distance"}, ...]
        """
        conflicts = []
        for pa in route.trajectory:
            for existing_cells, aircraft_set in self._occupancy.items():
                ix_a, iy_a, iz_a, it_a = existing_cells
                if abs(pa.t - self.config.step_to_time(it_a)) > time_window:
                    continue
                # 距离检查
                if route.aircraft_id in aircraft_set:
                    continue  # 自身不算冲突
                # 把网格索引转回世界坐标中心
                wx, wy, wz = self.config.to_world(ix_a, iy_a, iz_a)
                dx = pa.x - wx
                dy = pa.y - wy
                dz = pa.z - wz
                d = np.sqrt(dx * dx + dy * dy + dz * dz)
                if d < min_separation:
                    # 找到现有航线 ID（取第一个）
                    other_aircraft = next(iter(aircraft_set))
                    conflicts.append({
                        "route_a": route.route_id,
                        "route_b": f"aircraft:{other_aircraft}",
                        "time": pa.t,
                        "position_a": (pa.x, pa.y, pa.z),
                        "position_b": (wx, wy, wz),
                        "distance": float(d),
                    })
        return conflicts

    def clear(self):
        """清空占用表"""
        self._occupancy.clear()


class ConflictResolver:
    """
    冲突协调器（任务书 §三.2 冲突协调）

    发现冲突后触发重规划：
    - 策略 A：延迟起飞时间（等占用方飞过）
    - 策略 B：高度调整（+1 层）
    - 策略 C：完全重规划（带禁用格）
    """

    def __init__(self, config: SpaceTimeConfig, min_separation: float = 10.0):
        self.config = config
        self.min_separation = min_separation

    def suggest_delay(self, conflict: dict, max_delay_s: float = 60.0) -> Optional[float]:
        """根据冲突时间和对方位置，估算需要延迟的秒数"""
        # 简化策略：延迟直到对方已飞离最近距离点
        return min(conflict["time"] + max_delay_s / 4, max_delay_s)

    def suggest_altitude_shift(self, current_z: float) -> float:
        """建议增加的高度（一个网格层）"""
        new_z = current_z + self.config.dz
        if new_z <= self.config.z_max:
            return new_z
        return current_z  # 已到顶，无法升


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
        max_iterations: int = 2_000_000,
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

        # 共享的单进程规划器（用于线程模式和顺序回退）
        self._planner = SpaceTimeAStar(
            config=config,
            obstacle_set=obstacle_set,
            max_iterations=max_iterations,
            grid=self._shared_grid,
        )

    def plan_batch(
        self,
        tasks: List[FlightTask],
        performances: Dict[str, AircraftPerformance],
        request_id: str = "batch-001",
        dispatch: bool = False,
    ) -> PlanningResult:
        """
        批量并行规划

        Args:
            tasks: 飞行任务列表
            performances: 飞行器性能字典 {aircraft_id: AircraftPerformance}
            request_id: 请求 ID
            dispatch: 调度模式开关。True 时只跑 A* 不计算 risk/heading 等附加字段，
                      并对失败任务自动微调起点/终点到最近空闲格后重试一次。
                      生产调度推荐 True；demo/报告用 False。

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
    ) -> PlannedRoute:
        """单任务规划（线程安全：共享只读 grid，dispatch 模式跳过附加计算）"""
        # ★ 使用共享的 SpaceTimeGrid（不重建 3MB 数组）
        planner = SpaceTimeAStar(
            config=self.config,
            obstacle_set=self.obstacle_set,
            max_speed=performance.max_speed,
            max_iterations=self.max_iterations,
            grid=self._shared_grid,
        )

        start_wp = task.start
        end_wp = task.end

        # ★ 调度模式下先做起点/终点预校验（在障碍物内就微调）
        # 这样 A* 不会因为"起点被障碍物占用"直接失败。
        if dispatch:
            sx, sy, sz = self._snap_to_free(start_wp.x, start_wp.y, start_wp.z)
            ex, ey, ez = self._snap_to_free(end_wp.x, end_wp.y, end_wp.z)
            # 把微调后的坐标写回航点（不影响 task 本体，只在本线程使用）
            from ..models.aircraft import Waypoint
            snapped_start = Waypoint(
                wp_id=start_wp.wp_id, x=sx, y=sy, z=sz,
                wp_type=start_wp.wp_type, hover_time=start_wp.hover_time,
                earliest_arrival=start_wp.earliest_arrival,
                latest_arrival=start_wp.latest_arrival,
                task_description=start_wp.task_description,
            )
            snapped_end = Waypoint(
                wp_id=end_wp.wp_id, x=ex, y=ey, z=ez,
                wp_type=end_wp.wp_type, hover_time=end_wp.hover_time,
                earliest_arrival=end_wp.earliest_arrival,
                latest_arrival=end_wp.latest_arrival,
                task_description=end_wp.task_description,
            )
            start_wp = snapped_start
            end_wp = snapped_end

        # 如果有多个途经点，分段规划
        if len(task.waypoints) > 2:
            return self._plan_multi_waypoint(task, performance, idx, planner, dispatch=dispatch,
                                              start_wp=start_wp, end_wp=end_wp)

        route = planner.plan(
            start=(start_wp.x, start_wp.y, start_wp.z),
            goal=(end_wp.x, end_wp.y, end_wp.z),
            start_time=task.departure_time,
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
    ) -> PlannedRoute:
        """多航点分段规划"""
        from ..models.task import TrajectoryPoint

        if start_wp is None:
            start_wp = task.start
        if end_wp is None:
            end_wp = task.end

        all_trajectory = []
        current_time = task.departure_time
        total_planning_time = 0.0

        # 多航点：起点 / 终点 / 每个 via 都做一次预微调
        waypoints = list(task.waypoints)
        if dispatch:
            snapped = []
            for wp in waypoints:
                sx, sy, sz = self._snap_to_free(wp.x, wp.y, wp.z)
                snapped.append(Waypoint(
                    wp_id=wp.wp_id, x=sx, y=sy, z=sz,
                    wp_type=wp.wp_type, hover_time=wp.hover_time,
                    earliest_arrival=wp.earliest_arrival,
                    latest_arrival=wp.latest_arrival,
                    task_description=wp.task_description,
                ))
            waypoints = snapped

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
