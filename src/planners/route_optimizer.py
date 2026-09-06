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
from ..models.task import TrajectoryPoint, PlannedRoute
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
    ):
        self.config = config
        self.obstacle_set = obstacle_set
        self.performance = performance
        self.astar = SpaceTimeAStar(
            config=config,
            obstacle_set=obstacle_set,
            max_speed=performance.max_speed,
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
        if len(task.waypoints) > 2:
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

        # 碰撞检测：平滑后若穿过障碍物，回退到原始 A* 路径
        if self._check_collision(corrected):
            corrected = raw_trajectory
            algorithm_name = "RouteOptimizer(MultiObjective, raw-fallback)"
        else:
            algorithm_name = "RouteOptimizer(MultiObjective)"

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

        return all_traj

    def _smooth_trajectory(self, trajectory: List[TrajectoryPoint]) -> List[TrajectoryPoint]:
        """
        轨迹平滑：三次样条插值

        对原始网格路径进行空间平滑，减少锯齿，生成连续可飞轨迹。
        """
        if len(trajectory) < 4:
            return trajectory

        # 提取坐标
        coords = np.array([[p.x, p.y, p.z] for p in trajectory])
        times = np.array([p.t for p in trajectory])

        # 累积弧长参数化
        dists = np.zeros(len(coords))
        for i in range(1, len(coords)):
            dists[i] = dists[i - 1] + np.linalg.norm(coords[i] - coords[i - 1])

        if dists[-1] < 1e-6:
            return trajectory

        # 归一化参数
        s = dists / dists[-1]

        # 三次样条插值（增加采样点）
        num_points = min(max(len(trajectory) * 2, 20), 200)
        s_new = np.linspace(0, 1, num_points)

        try:
            cs_x = CubicSpline(s, coords[:, 0])
            cs_y = CubicSpline(s, coords[:, 1])
            cs_z = CubicSpline(s, coords[:, 2])

            x_new = cs_x(s_new)
            y_new = cs_y(s_new)
            z_new = cs_z(s_new)
        except Exception:
            # 插值失败，返回原始轨迹
            return trajectory

        # 时间重新分配（按距离 / 平均速度）
        avg_speed = self.performance.max_speed * 0.7  # 巡航速度取 70% 最大速度
        total_dist = dists[-1]
        total_time = total_dist / avg_speed

        smoothed = []
        for i in range(num_points):
            t = task_departure = trajectory[0].t + (s_new[i] * total_time)
            # 计算航向
            if i > 0:
                dx = x_new[i] - x_new[i - 1]
                dy = y_new[i] - y_new[i - 1]
                heading = float(np.degrees(np.arctan2(dy, dx)))
            else:
                heading = trajectory[0].heading

            # 高度约束
            z = np.clip(z_new[i], self.performance.min_altitude, self.performance.max_altitude)

            smoothed.append(TrajectoryPoint(
                x=float(x_new[i]),
                y=float(y_new[i]),
                z=float(z),
                t=float(t),
                heading=heading,
            ))

        return smoothed

    def _enforce_performance_constraints(self, trajectory: List[TrajectoryPoint]) -> List[TrajectoryPoint]:
        """
        强制执行飞行器性能约束

        检查并修正:
        - 最大速度
        - 最大爬升/下降率
        - 最大转弯率
        """
        if len(trajectory) < 2:
            return trajectory

        corrected = [trajectory[0]]

        for i in range(1, len(trajectory)):
            p_prev = corrected[-1]
            p_curr = trajectory[i]

            dt = p_curr.t - p_prev.t
            if dt <= 0:
                continue

            dx = p_curr.x - p_prev.x
            dy = p_curr.y - p_prev.y
            dz = p_curr.z - p_prev.z

            vx = dx / dt
            vy = dy / dt
            vz = dz / dt

            # 水平速度约束
            horizontal_speed = np.sqrt(vx * vx + vy * vy)
            if horizontal_speed > self.performance.max_speed:
                scale = self.performance.max_speed / horizontal_speed
                vx *= scale
                vy *= scale
                dx = vx * dt
                dy = vy * dt

            # 垂直速度约束
            if vz > self.performance.max_climb_rate:
                vz = self.performance.max_climb_rate
                dz = vz * dt
            elif vz < -self.performance.max_descent_rate:
                vz = -self.performance.max_descent_rate
                dz = vz * dt

            # 转弯率约束（简化：限制航向变化）
            new_heading = float(np.degrees(np.arctan2(vy, vx))) if (abs(vx) > 1e-6 or abs(vy) > 1e-6) else p_prev.heading
            heading_diff = ((new_heading - p_prev.heading + 180) % 360) - 180
            max_heading_change = self.performance.max_turn_rate * dt
            if abs(heading_diff) > max_heading_change:
                heading_diff = np.sign(heading_diff) * max_heading_change
                new_heading = p_prev.heading + heading_diff

            corrected.append(TrajectoryPoint(
                x=p_prev.x + dx,
                y=p_prev.y + dy,
                z=np.clip(p_prev.z + dz, self.performance.min_altitude, self.performance.max_altitude),
                t=p_curr.t,
                vx=vx,
                vy=vy,
                vz=vz,
                heading=new_heading,
            ))

        return corrected

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
        """检查轨迹是否与静态障碍物碰撞"""
        for p in trajectory:
            for obs in self.obstacle_set.static_obstacles:
                if (
                    abs(p.x - obs.x) <= obs.width
                    and abs(p.y - obs.y) <= obs.depth
                    and obs.z <= p.z <= obs.z + obs.height
                ):
                    return True
        return False

    def _explain_result(self, route: PlannedRoute, task: FlightTask) -> str:
        """
        生成结果解释（3.26 模块四: 航点生成与结果解释）

        分析各优化目标的贡献度，给出人类可读的规划说明。
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
