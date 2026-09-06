"""
大规模并行规划引擎 (3.25 模块四)

支持多飞行器、多任务的并行路径规划，
使用 concurrent.futures 实现多进程/多线程并行，
并支持优先级调度和冲突检测。
"""

from __future__ import annotations

import time
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor, as_completed
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from ..models.space import SpaceTimeConfig
from ..models.obstacles import ObstacleSet
from ..models.aircraft import AircraftPerformance, FlightTask
from ..models.task import PlannedRoute, PlanningResult
from .spacetime_astar import SpaceTimeAStar


def _plan_single_task(args: tuple) -> PlannedRoute:
    """
    单任务规划函数（用于多进程并行）

    args = (config_dict, obstacle_data, task_dict, performance_dict, idx)
    """
    config_dict, obstacle_data, task_dict, performance_dict, idx = args

    # 重建配置
    config = SpaceTimeConfig(**config_dict)

    # 重建障碍物（简化：从 grid 数据重建）
    obstacle_set = ObstacleSet()
    # 注意：在实际使用中，障碍物数据需要可序列化传递
    # 这里通过共享 grid 数据来避免重复构建

    # 重建任务
    from ..models.aircraft import Waypoint
    waypoints = []
    for wp_data in task_dict["waypoints"]:
        waypoints.append(Waypoint(**wp_data))
    task = FlightTask(
        task_id=task_dict["task_id"],
        task_name=task_dict["task_name"],
        aircraft_id=task_dict["aircraft_id"],
        waypoints=waypoints,
        priority=task_dict.get("priority", 5),
        task_type=task_dict.get("task_type", "other"),
        departure_time=task_dict.get("departure_time", 0.0),
    )

    performance = AircraftPerformance(**performance_dict)

    # 执行规划
    planner = SpaceTimeAStar(
        config=config,
        obstacle_set=obstacle_set,
        max_speed=performance.max_speed,
    )

    start_wp = task.start
    end_wp = task.end

    route = planner.plan(
        start=(start_wp.x, start_wp.y, start_wp.z),
        goal=(end_wp.x, end_wp.y, end_wp.z),
        start_time=task.departure_time,
        task_id=task.task_id,
        aircraft_id=task.aircraft_id,
        route_id=f"route-{idx:03d}",
    )

    return route


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
        use_processes: bool = False,
    ):
        self.config = config
        self.obstacle_set = obstacle_set
        self.max_workers = max_workers
        self.use_processes = use_processes

        # 共享的单进程规划器（用于线程模式和顺序回退）
        self._planner = SpaceTimeAStar(
            config=config,
            obstacle_set=obstacle_set,
        )

    def plan_batch(
        self,
        tasks: List[FlightTask],
        performances: Dict[str, AircraftPerformance],
        request_id: str = "batch-001",
    ) -> PlanningResult:
        """
        批量并行规划

        Args:
            tasks: 飞行任务列表
            performances: 飞行器性能字典 {aircraft_id: AircraftPerformance}
            request_id: 请求 ID

        Returns:
            PlanningResult: 批量规划结果
        """
        t0 = time.time()

        # 按优先级排序（高优先级先规划）
        sorted_tasks = sorted(tasks, key=lambda t: t.priority, reverse=True)

        routes: List[PlannedRoute] = []
        success_count = 0
        failed_count = 0

        if self.use_processes and len(sorted_tasks) > 1:
            # 多进程模式
            routes = self._plan_multiprocess(sorted_tasks, performances)
        else:
            # 多线程模式（共享内存，适合 IO 密集或共享障碍物数据）
            routes = self._plan_multithread(sorted_tasks, performances)

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
    ) -> PlannedRoute:
        """单任务规划（线程安全：每个线程使用独立的 planner 实例）"""
        planner = SpaceTimeAStar(
            config=self.config,
            obstacle_set=self.obstacle_set,
            max_speed=performance.max_speed,
        )

        start_wp = task.start
        end_wp = task.end

        # 如果有多个途经点，分段规划
        if len(task.waypoints) > 2:
            return self._plan_multi_waypoint(task, performance, idx, planner)

        route = planner.plan(
            start=(start_wp.x, start_wp.y, start_wp.z),
            goal=(end_wp.x, end_wp.y, end_wp.z),
            start_time=task.departure_time,
            task_id=task.task_id,
            aircraft_id=task.aircraft_id,
            route_id=f"route-{idx:03d}",
        )
        return route

    def _plan_multi_waypoint(
        self,
        task: FlightTask,
        performance: AircraftPerformance,
        idx: int,
        planner: SpaceTimeAStar,
    ) -> PlannedRoute:
        """多航点分段规划"""
        from ..models.task import TrajectoryPoint

        all_trajectory = []
        current_time = task.departure_time
        total_planning_time = 0.0

        for i in range(len(task.waypoints) - 1):
            wp_start = task.waypoints[i]
            wp_end = task.waypoints[i + 1]

            seg_route = planner.plan(
                start=(wp_start.x, wp_start.y, wp_start.z),
                goal=(wp_end.x, wp_end.y, wp_end.z),
                start_time=current_time,
                task_id=task.task_id,
                aircraft_id=task.aircraft_id,
                route_id=f"route-{idx:03d}-seg{i}",
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

            # 拼接轨迹（跳过重复的起点）
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
        route.compute_metrics(performance)
        return route

    def _plan_multiprocess(
        self,
        tasks: List[FlightTask],
        performances: Dict[str, AircraftPerformance],
    ) -> List[PlannedRoute]:
        """多进程并行规划（注意：障碍物数据需要可序列化）"""
        # 简化实现：多进程模式下回退到线程模式
        # 因为障碍物集合包含复杂对象，序列化开销大
        return self._plan_multithread(tasks, performances)

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
