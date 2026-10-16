"""
多目标航线优化器 (3.26 模块三)

基于加权求和的多目标航线优化，优化目标包括:
- 航程最短 (distance)
- 飞行时间最短 (time)
- 风险最低 (risk) - 距障碍物距离
- 能耗最低 (energy)

支持:
- 多目标加权优化
- 航点生成与轨迹平滑
- 结果解释（各目标贡献度分析）
- 飞行器性能约束满足
"""

from __future__ import annotations

import time
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.interpolate import CubicSpline

from ..models.space import SpaceTimeConfig
from ..models.obstacles import ObstacleSet
from ..models.aircraft import AircraftPerformance, FlightTask, Waypoint
from ..models.task import TrajectoryPoint, PlannedRoute, PlanningExplanation
from ..grid.cost import CostGrid
from ..validator.path_validator import PathValidator
from .spacetime_astar import SpaceTimeAStar


class RouteOptimizer:
    """
    智能飞行线路规划 - 多目标航线优化器

    基于已有 A* 路径进行多目标优化和平滑，
    满足飞行器性能约束，生成可执行的航点序列。
    """

    def __init__(
        self,
        config: SpaceTimeConfig,
        obstacle_set: ObstacleSet,
        performance: AircraftPerformance,
        cost_grid: Optional[CostGrid] = None,
    ):
        self.config = config
        self.obstacle_set = obstacle_set
        self.performance = performance
        self.cost_grid = cost_grid
        self.astar = SpaceTimeAStar(
            config=config,
            obstacle_set=obstacle_set,
            max_speed=performance.get_max_speed(),
            cost_grid=cost_grid,
            min_altitude=performance.get_min_altitude(),
            max_altitude=performance.get_max_altitude(),
            max_climb_rate=performance.get_max_climb_rate(),
            max_descent_rate=performance.get_max_descent_rate(),
        )

    def optimize(
        self,
        task: FlightTask,
        route_id: str = "opt-route-001",
    ) -> PlannedRoute:
        """
        执行多目标航线优化

        流程:
        1. 用时空 A* 生成初始路径
        2. 轨迹平滑（三次样条插值）
        3. 多目标代价评估
        4. 航点提取与结果解释

        Args:
            task: 飞行任务
            route_id: 航线 ID

        Returns:
            PlannedRoute: 优化后的航线
        """
        t0 = time.time()

        # 验证任务
        valid, msg = task.validate()
        if not valid:
            return PlannedRoute(
                route_id=route_id,
                task_id=task.task_id,
                aircraft_id=task.aircraft_id,
                status="failed",
                message=f"任务验证失败: {msg}",
                algorithm="RouteOptimizer",
                planning_time_ms=(time.time() - t0) * 1000,
            )

        # 多航点分段规划
        if len(task.waypoints) > 2 or any(wp.hover_time > 0 for wp in task.waypoints):
            raw_trajectory = self._plan_segments(task)
        else:
            raw_route = self.astar.plan(
                start=(task.start.x, task.start.y, task.start.z),
                goal=(task.end.x, task.end.y, task.end.z),
                start_time=task.departure_time,
                task_id=task.task_id,
                aircraft_id=task.aircraft_id,
                route_id=route_id + "-raw",
            )
            if raw_route.status != "planned":
                raw_route.algorithm = "RouteOptimizer(A* failed)"
                return raw_route
            raw_trajectory = raw_route.trajectory

        if not raw_trajectory:
            return PlannedRoute(
                route_id=route_id,
                task_id=task.task_id,
                aircraft_id=task.aircraft_id,
                status="failed",
                message="初始路径规划失败",
                algorithm="RouteOptimizer",
                planning_time_ms=(time.time() - t0) * 1000,
            )

        # 轨迹平滑
        smoothed = self._smooth_trajectory(raw_trajectory)

        # 性能约束校验与修正
        corrected = self._enforce_performance_constraints(smoothed)

        # A geometry candidate must retain the schedule and pass the same full
        # validation as the raw fallback. Retiming or clipping coordinates can
        # invalidate obstacle avoidance, waits, endpoints, and intermediate tasks.
        if not self._candidate_is_valid(corrected, task):
            corrected = self._enforce_performance_constraints(raw_trajectory)
            algorithm_name = "RouteOptimizer(MultiObjective, raw-fallback)"
        else:
            algorithm_name = "RouteOptimizer(MultiObjective)"
        if not self._candidate_is_valid(corrected, task):
            return PlannedRoute(route_id=route_id, task_id=task.task_id, aircraft_id=task.aircraft_id,
                trajectory=corrected, status="failed", message="路径未通过完整安全与性能验证",
                algorithm="RouteOptimizer(validation failed)", planning_time_ms=(time.time() - t0) * 1000)

        # 计算多目标指标
        route = PlannedRoute(
            route_id=route_id,
            task_id=task.task_id,
            aircraft_id=task.aircraft_id,
            trajectory=corrected,
            status="planned",
            algorithm=algorithm_name,
            planning_time_ms=(time.time() - t0) * 1000,
        )
        route.compute_metrics(self.performance)

        # 风险评估
        self._evaluate_risk(route)

        # 多目标加权得分
        route.message = self._explain_result(route, task)

        return route

    def _plan_segments(self, task: FlightTask) -> List[TrajectoryPoint]:
        """多航点分段规划"""
        all_traj = []
        current_time = task.departure_time
        if task.start.hover_time > 0:
            all_traj.append(TrajectoryPoint(x=task.start.x, y=task.start.y, z=task.start.z, t=current_time))
            current_time += task.start.hover_time
            all_traj.append(TrajectoryPoint(x=task.start.x, y=task.start.y, z=task.start.z, t=current_time))

        for i in range(len(task.waypoints) - 1):
            wp_s = task.waypoints[i]
            wp_e = task.waypoints[i + 1]

            seg = self.astar.plan(
                start=(wp_s.x, wp_s.y, wp_s.z),
                goal=(wp_e.x, wp_e.y, wp_e.z),
                start_time=current_time,
                task_id=task.task_id,
                aircraft_id=task.aircraft_id,
                route_id=f"seg-{i}",
            )

            if seg.status != "planned":
                return []

            if all_traj:
                all_traj.extend(seg.trajectory[1:])
            else:
                all_traj.extend(seg.trajectory)

            current_time = seg.trajectory[-1].t + wp_e.hover_time
            if wp_e.hover_time > 0:
                all_traj.append(TrajectoryPoint(x=wp_e.x, y=wp_e.y, z=wp_e.z, t=current_time))

        return all_traj

    def _smooth_trajectory(self, trajectory: List[TrajectoryPoint]) -> List[TrajectoryPoint]:
        """Smooth positions at fixed times, retaining all original samples.

        Wait intervals stay verbatim: cubic interpolation could move the vehicle
        between equal-position endpoints during a required dwell.
        """
        if len(trajectory) < 4:
            return trajectory
        coords = np.array([[p.x, p.y, p.z] for p in trajectory])
        times = np.array([p.t for p in trajectory])
        if np.any(np.diff(times) <= 0) or np.any(np.linalg.norm(np.diff(coords, axis=0), axis=1) <= 1e-9):
            return trajectory
        # Keeping every original time also retains intermediate task positions.
        sample_times = np.unique(np.concatenate((times, (times[:-1] + times[1:]) / 2)))
        try:
            spline = CubicSpline(times, coords, axis=0)
            positions = spline(sample_times)
        except ValueError:
            return trajectory
        originals = {p.t: p for p in trajectory}
        result = []
        for timestamp, xyz in zip(sample_times, positions):
            original = originals.get(float(timestamp))
            if original is not None:
                xyz = (original.x, original.y, original.z)
            result.append(TrajectoryPoint(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]), t=float(timestamp)))
        return result

    def _enforce_performance_constraints(self, trajectory: List[TrajectoryPoint]) -> List[TrajectoryPoint]:
        """Populate truthful motion metadata without changing geometry or time.

        Feasibility is decided by validation. Changing coordinates to limit a
        computed speed could silently move the endpoint or cut an obstacle.
        """
        result = []
        for i, point in enumerate(trajectory):
            vx = vy = vz = 0.0
            heading = point.heading if not result else result[-1].heading
            if i:
                previous = trajectory[i - 1]
                duration = point.t - previous.t
                if duration > 0:
                    vx, vy, vz = ((point.x - previous.x) / duration,
                                  (point.y - previous.y) / duration,
                                  (point.z - previous.z) / duration)
                    if np.hypot(vx, vy) > 1e-9:
                        heading = float(np.degrees(np.arctan2(vy, vx)))
            result.append(TrajectoryPoint(x=point.x, y=point.y, z=point.z, t=point.t,
                                          vx=vx, vy=vy, vz=vz, heading=heading))
        return result

    def _candidate_is_valid(self, trajectory: List[TrajectoryPoint], task: FlightTask) -> bool:
        if len(trajectory) < 2 or trajectory[0].t != task.departure_time:
            return False
        if ((trajectory[0].x, trajectory[0].y, trajectory[0].z) != (task.start.x, task.start.y, task.start.z)
                or (trajectory[-1].x, trajectory[-1].y, trajectory[-1].z) != (task.end.x, task.end.y, task.end.z)):
            return False
        candidate = PlannedRoute(route_id="candidate", task_id=task.task_id,
                                 aircraft_id=task.aircraft_id, trajectory=trajectory, status="planned")
        candidate.compute_metrics(self.performance)
        if (candidate.total_time > self.performance.get_max_flight_time() + 1e-6
                or candidate.total_distance > self.performance.get_max_range() + 1e-6):
            return False
        validation = PathValidator(obstacle_set=self.obstacle_set, grid=self.astar.grid).validate_route(
            candidate, self.performance)
        if not validation.is_valid:
            return False
        cursor = 0
        for waypoint in task.waypoints:
            found = None
            for index in range(cursor, len(trajectory)):
                point = trajectory[index]
                if (point.x, point.y, point.z) == (waypoint.x, waypoint.y, waypoint.z):
                    found = index
                    break
            if found is None:
                return False
            arrival = trajectory[found].t
            if (waypoint.earliest_arrival is not None and arrival < waypoint.earliest_arrival - 1e-6
                    or waypoint.latest_arrival is not None and arrival > waypoint.latest_arrival + 1e-6):
                return False
            cursor = found
            if waypoint.hover_time > 0:
                target = arrival + waypoint.hover_time
                while cursor + 1 < len(trajectory) and trajectory[cursor].t < target - 1e-6:
                    cursor += 1
                    if (trajectory[cursor].x, trajectory[cursor].y, trajectory[cursor].z) != (waypoint.x, waypoint.y, waypoint.z):
                        return False
                if trajectory[cursor].t < target - 1e-6:
                    return False
        return True

    def _evaluate_risk(self, route: PlannedRoute):
        """评估航线风险（距障碍物距离）"""
        if not route.trajectory:
            return

        min_dist = float("inf")
        risks = []

        for p in route.trajectory:
            point_min_dist = float("inf")

            # 静态障碍物 - 使用包围盒距离
            for obs in self.obstacle_set.static_obstacles:
                dx = max(abs(p.x - obs.x) - obs.width, 0.0)
                dy = max(abs(p.y - obs.y) - obs.depth, 0.0)
                dz = max(max(p.z - (obs.z + obs.height), 0.0), max(obs.z - p.z, 0.0))
                dist = np.sqrt(dx * dx + dy * dy + dz * dz)
                point_min_dist = min(point_min_dist, dist)

            # 动态障碍物
            for obs in self.obstacle_set.dynamic_obstacles:
                pos = obs.position_at(p.t)
                if pos is not None:
                    dx = p.x - pos[0]
                    dy = p.y - pos[1]
                    dz = p.z - pos[2]
                    dist = np.sqrt(dx * dx + dy * dy + dz * dz) - obs.safety_radius
                    point_min_dist = min(point_min_dist, dist)

            min_dist = min(min_dist, point_min_dist)
            # 风险值: 距离 0m → 风险 1.0, 距离 50m+ → 风险 0.0，钳制在 [0,1]
            risk = np.clip(1.0 - point_min_dist / 50.0, 0.0, 1.0) if point_min_dist != float("inf") else 0.0
            risks.append(risk)

        route.min_obstacle_distance = max(0.0, min_dist) if min_dist != float("inf") else 999.0
        route.max_risk = max(risks) if risks else 0.0
        route.avg_risk = float(np.mean(risks)) if risks else 0.0

    def _check_collision(self, trajectory: List[TrajectoryPoint]) -> bool:
        """Check continuous static/dynamic obstacles and grid occupancy."""
        route = PlannedRoute(route_id="collision-check", task_id="collision-check", aircraft_id=self.performance.aircraft_id,
                             trajectory=trajectory, status="planned")
        return not PathValidator(obstacle_set=self.obstacle_set, grid=self.astar.grid).validate_route(route).is_valid

    def _explain_result(self, route: PlannedRoute, task: FlightTask) -> str:
        """
        生成结果解释 (3.26 模块四: 航点生成与结果解释)

        分析各优化目标的贡献度，给出人类可读的规划说明，
        同时把结构化解释挂到 ``route.explanation``（任务书 §十 输出要求
        "规划解释"字段），供下游服务直接消费。
        """
        weights = task.objective_weights

        # 归一化各指标（用于计算加权得分）
        norm_distance = min(1.0, route.total_distance / 10000.0)
        norm_time = min(1.0, route.total_time / 600.0)
        norm_risk = route.avg_risk
        norm_energy = min(1.0, route.total_energy / 10000.0)

        weighted_score = (
            weights.get("distance", 0.25) * norm_distance
            + weights.get("time", 0.25) * norm_time
            + weights.get("risk", 0.25) * norm_risk
            + weights.get("energy", 0.25) * norm_energy
        )

        # ---- 结构化解释（PlanningExplanation）----
        breakdown = {
            "distance": weights.get("distance", 0.25) * norm_distance,
            "time": weights.get("time", 0.25) * norm_time,
            "risk": weights.get("risk", 0.25) * norm_risk,
            "energy": weights.get("energy", 0.25) * norm_energy,
        }
        margin = self._compute_performance_margin(route.trajectory)

        notes = []
        if route.algorithm.endswith("raw-fallback"):
            notes.append("平滑候选未通过完整安全验证，已回退为原始 A* 路径")
        if route.max_risk > 0.5:
            notes.append(f"存在贴近障碍物的航段（最大风险 {route.max_risk:.2f}），建议复核")
        if margin.get("turn_rate_ratio", 0) > 0.9:
            notes.append("转弯率接近飞行器极限，建议降低航点密度或放宽权重")
        if route.total_time > self.performance.get_max_flight_time() * 0.8:
            notes.append(
                f"飞行时间 {route.total_time:.0f}s 已达续航 "
                f"{self.performance.get_max_flight_time():.0f}s 的 80%，余量偏小"
            )

        route.explanation = PlanningExplanation(
            summary=(
                f"四维时空 A* 生成 {len(route.trajectory)} 点初始航路，经候选平滑和完整"
                f"安全验证后，综合得分 {weighted_score:.4f}（越低越优）"
            ),
            score=weighted_score,
            objective_breakdown=breakdown,
            avoidance_count=self._count_avoidances(route.trajectory),
            performance_margin=margin,
            notes=notes,
        )

        explanation = (
            f"规划完成 | 综合得分: {weighted_score:.4f} (越低越优) | "
            f"航程: {route.total_distance:.1f}m (权重{weights.get('distance', 0):.0%}) | "
            f"时间: {route.total_time:.1f}s (权重{weights.get('time', 0):.0%}) | "
            f"平均风险: {route.avg_risk:.4f} (权重{weights.get('risk', 0):.0%}) | "
            f"能耗: {route.total_energy:.1f} (权重{weights.get('energy', 0):.0%}) | "
            f"最近障碍距离: {route.min_obstacle_distance:.1f}m | "
            f"航点数: {len(route.trajectory)}"
        )

        return explanation

    def _compute_performance_margin(self, trajectory: List[TrajectoryPoint]) -> dict:
        """计算各性能指标的极限占用比例（1.0 = 打满极限，>1 = 超限）"""
        max_speed = self.performance.get_max_speed()
        max_climb = self.performance.get_max_climb_rate()
        max_desc = self.performance.get_max_descent_rate()
        max_turn = self.performance.performance.max_turn_rate_dps

        max_v_h = 0.0
        max_climb_used = 0.0
        max_desc_used = 0.0
        max_turn_used = 0.0

        for i in range(1, len(trajectory)):
            p0, p1 = trajectory[i - 1], trajectory[i]
            dt = p1.t - p0.t
            if dt <= 0:
                continue
            vx = (p1.x - p0.x) / dt
            vy = (p1.y - p0.y) / dt
            vz = (p1.z - p0.z) / dt
            max_v_h = max(max_v_h, float(np.hypot(vx, vy)))
            max_climb_used = max(max_climb_used, vz)
            max_desc_used = max(max_desc_used, -vz)
            if i >= 2:
                p_prev = trajectory[i - 2]
                h0 = np.degrees(np.arctan2(p0.y - p_prev.y, p0.x - p_prev.x))
                h1 = np.degrees(np.arctan2(p1.y - p0.y, p1.x - p0.x))
                d_h = ((h1 - h0 + 180) % 360) - 180
                max_turn_used = max(max_turn_used, abs(d_h) / dt)

        def ratio(used: float, limit: float) -> float:
            return float(used / limit) if limit > 0 else 0.0

        return {
            "speed_ratio": ratio(max_v_h, max_speed),
            "climb_ratio": ratio(max_climb_used, max_climb),
            "descent_ratio": ratio(max_desc_used, max_desc),
            "turn_rate_ratio": ratio(max_turn_used, max_turn),
        }

    def _count_avoidances(self, trajectory: List[TrajectoryPoint], radius: float = 50.0) -> int:
        """统计航线贴近（< radius 米）的障碍物数量，即实际规避过的障碍物数"""
        if not trajectory or not self.obstacle_set:
            return 0
        avoided = set()
        for p in trajectory:
            for obs in self.obstacle_set.static_obstacles:
                dx = max(abs(p.x - obs.x) - obs.width, 0.0)
                dy = max(abs(p.y - obs.y) - obs.depth, 0.0)
                dz = max(max(p.z - (obs.z + obs.height), 0.0), max(obs.z - p.z, 0.0))
                if np.sqrt(dx * dx + dy * dy + dz * dz) < radius:
                    avoided.add(obs.obstacle_id)
            for obs in self.obstacle_set.dynamic_obstacles:
                pos = obs.position_at(p.t)
                if pos is None:
                    continue
                d = np.sqrt(
                    (p.x - pos[0]) ** 2 + (p.y - pos[1]) ** 2 + (p.z - pos[2]) ** 2
                )
                if d - obs.safety_radius < radius:
                    avoided.add(obs.obstacle_id)
        return len(avoided)

    def generate_waypoints(
        self,
        trajectory: List[TrajectoryPoint],
        min_spacing: float = 50.0,
    ) -> List[dict]:
        """
        从连续轨迹提取关键航点 (3.26 模块四)

        按最小间距和航向变化提取航点，生成可执行的航点指令。
        """
        if not trajectory:
            return []

        waypoints = []
        last_wp_pos = None

        for i, p in enumerate(trajectory):
            if last_wp_pos is None:
                waypoints.append({
                    "seq": len(waypoints),
                    "x": round(p.x, 2),
                    "y": round(p.y, 2),
                    "z": round(p.z, 2),
                    "t": round(p.t, 2),
                    "heading": round(p.heading, 1),
                    "action": "takeoff" if i == 0 else "fly",
                })
                last_wp_pos = (p.x, p.y, p.z)
                continue

            dx = p.x - last_wp_pos[0]
            dy = p.y - last_wp_pos[1]
            dz = p.z - last_wp_pos[2]
            dist = np.sqrt(dx * dx + dy * dy + dz * dz)

            # 检查航向变化
            heading_changed = False
            if waypoints:
                last_heading = waypoints[-1]["heading"]
                heading_diff = abs(((p.heading - last_heading + 180) % 360) - 180)
                heading_changed = heading_diff > 15.0

            if dist >= min_spacing or heading_changed or i == len(trajectory) - 1:
                action = "land" if i == len(trajectory) - 1 else "fly"
                waypoints.append({
                    "seq": len(waypoints),
                    "x": round(p.x, 2),
                    "y": round(p.y, 2),
                    "z": round(p.z, 2),
                    "t": round(p.t, 2),
                    "heading": round(p.heading, 1),
                    "action": action,
                })
                last_wp_pos = (p.x, p.y, p.z)

        return waypoints
