"""
多维时空 A* 路径规划算法 (3.25 核心算法)

在 (x, y, z, t) 四维时空网格中进行 A* 搜索，
支持静态障碍物（所有时间步占用）和动态障碍物（特定时间步占用），
实现时空联合避障。

算法特点:
- 四维状态空间 (ix, iy, iz, it)
- 26 邻域空间移动 + 等待动作（时间推进）
- 启发函数: 欧氏距离 / 最大速度
- 支持动态障碍物时空占用检测
"""

from __future__ import annotations

import heapq
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

from ..models.space import SpaceTimeConfig, SpaceTimeGrid
from ..models.obstacles import ObstacleSet
from ..models.task import TrajectoryPoint, PlannedRoute


class SpaceTimeAStar:
    """
    多维时空 A* 规划器

    在四维时空网格中搜索从起点到终点的无碰撞路径。
    """

    # 6 邻域（面连接）+ 等待动作，分支因子低，搜索效率高
    NEIGHBORS_6 = [
        (1, 0, 0), (-1, 0, 0),
        (0, 1, 0), (0, -1, 0),
        (0, 0, 1), (0, 0, -1),
    ]

    def __init__(
        self,
        config: SpaceTimeConfig,
        obstacle_set: Optional[ObstacleSet] = None,
        max_speed: float = 15.0,
        wait_enabled: bool = True,
        neighbor_mode: str = "6",
        max_iterations: int = 2_000_000,
        grid: Optional[SpaceTimeGrid] = None,
    ):
        self.config = config
        self.obstacle_set = obstacle_set
        self.max_speed = max_speed
        self.wait_enabled = wait_enabled
        self.neighbor_mode = neighbor_mode
        self.max_iterations = max_iterations

        if neighbor_mode == "26":
            self.neighbors = [
                (dx, dy, dz)
                for dx in (-1, 0, 1)
                for dy in (-1, 0, 1)
                for dz in (-1, 0, 1)
                if not (dx == 0 and dy == 0 and dz == 0)
            ]
        else:
            self.neighbors = self.NEIGHBORS_6

        # 网格：优先使用外部传入的共享 grid，否则自建
        # 这样 ParallelPlanner 可以共享一份 3MB 占用网格，
        # 消除 10000 个任务各自重建时空网格的浪费。
        if grid is not None:
            self.grid = grid
        else:
            self.grid = SpaceTimeGrid(config)
            if obstacle_set is not None:
                self._build_grid_from_obstacles()

    def _build_grid_from_obstacles(self):
        """将障碍物集合写入时空占用网格"""
        # 静态障碍物（使用精确包围盒）
        for obs in self.obstacle_set.static_obstacles:
            self.grid.set_static_box(
                x_min=obs.x - obs.width,
                x_max=obs.x + obs.width,
                y_min=obs.y - obs.depth,
                y_max=obs.y + obs.depth,
                z_min=obs.z,
                z_max=obs.z + obs.height,
            )

        # 动态障碍物
        for obs in self.obstacle_set.dynamic_obstacles:
            self.grid.set_dynamic_obstacle(obs.trajectory, obs.safety_radius)

    def _heuristic(self, state: Tuple[int, int, int, int], goal: Tuple[int, int, int, int]) -> float:
        """
        启发函数: 曼哈顿网格距离 * 时间步长
        估计到达目标所需的最少时间步数，保证可采纳性。
        """
        ix, iy, iz, it = state
        gx, gy, gz, gt = goal
        manhattan = abs(ix - gx) + abs(iy - gy) + abs(iz - gz)
        return manhattan * self.config.dt

    def _get_neighbors(
        self, state: Tuple[int, int, int, int]
    ) -> List[Tuple[Tuple[int, int, int, int], float]]:
        """
        获取当前状态的所有可行后继状态及代价
        返回: [(next_state, cost), ...]
        """
        ix, iy, iz, it = state
        neighbors = []

        # 空间移动（6/26 邻域）
        for dx, dy, dz in self.neighbors:
            nix, niy, niz = ix + dx, iy + dy, iz + dz
            nit = it + 1  # 每次移动推进一个时间步

            # 边界检查
            if not (
                0 <= nix < self.config.nx
                and 0 <= niy < self.config.ny
                and 0 <= niz < self.config.nz
                and 0 <= nit < self.config.nt
            ):
                continue

            # 速度约束检查（空间位移 / 时间步 <= 最大速度）
            spatial_dist = np.sqrt(
                (dx * self.config.dx) ** 2
                + (dy * self.config.dy) ** 2
                + (dz * self.config.dz) ** 2
            )
            if spatial_dist / self.config.dt > self.max_speed + 1e-6:
                continue

            # 碰撞检查
            if not self.grid.is_free(nix, niy, niz, nit):
                continue

            # 代价 = 时间步长（每次移动消耗 dt 时间）+ 微小距离惩罚（打破平局，偏好短路径）
            cost = self.config.dt + spatial_dist * 0.001
            neighbors.append(((nix, niy, niz, nit), cost))

        # 等待动作（原地停留，时间推进）
        if self.wait_enabled:
            nit = it + 1
            if nit < self.config.nt and self.grid.is_free(ix, iy, iz, nit):
                cost = self.config.dt * 1.05  # 等待略贵于移动，鼓励前进
                neighbors.append(((ix, iy, iz, nit), cost))

        return neighbors

    def plan(
        self,
        start: Tuple[float, float, float],
        goal: Tuple[float, float, float],
        start_time: float = 0.0,
        task_id: str = "task-001",
        aircraft_id: str = "uav-001",
        route_id: str = "route-001",
        extras: bool = True,
    ) -> PlannedRoute:
        """
        执行多维时空 A* 规划

        Args:
            start: 起点 (x, y, z)
            goal: 终点 (x, y, z)
            start_time: 起始时间 (秒)
            task_id: 任务 ID
            aircraft_id: 飞行器 ID
            route_id: 航线 ID
            extras: 是否计算风险指标、航向角等额外信息。
                    设为 False 时进入"调度模式"，跳过 _compute_risk 和
                    heading 计算（实测节省 ~8% A* 总耗时，仅返回必要字段）。

        Returns:
            PlannedRoute: 规划结果
        """
        t0 = time.time()

        # 转换为网格坐标
        six, siy, siz = self.config.to_grid(*start)
        gix, giy, giz = self.config.to_grid(*goal)
        sit = self.config.time_to_step(start_time)

        start_state = (six, siy, siz, sit)
        goal_state = (gix, giy, giz, -1)  # 时间步不固定，到达空间即可

        # 检查起点终点是否可行
        if not self.grid.is_free(six, siy, siz, sit):
            return PlannedRoute(
                route_id=route_id,
                task_id=task_id,
                aircraft_id=aircraft_id,
                status="failed",
                message="起点被障碍物占用",
                algorithm="SpaceTimeAStar",
                planning_time_ms=(time.time() - t0) * 1000,
            )

        # A* 搜索
        open_set: List[Tuple[float, int, Tuple[int, int, int, int]]] = []
        counter = 0
        h_start = self._heuristic(start_state, (gix, giy, giz, 0))
        heapq.heappush(open_set, (h_start, counter, start_state))

        came_from: Dict[Tuple[int, int, int, int], Tuple[int, int, int, int]] = {}
        g_score: Dict[Tuple[int, int, int, int], float] = {start_state: 0.0}
        closed_set: set = set()

        max_iterations = self.max_iterations
        iterations = 0
        found = False
        current = start_state

        while open_set and iterations < max_iterations:
            iterations += 1
            f_current, _, current = heapq.heappop(open_set)

            # 跳过过期条目（该状态已被更优路径扩展过）
            if current in closed_set:
                continue
            if current in g_score and f_current > g_score[current] + self._heuristic(current, (gix, giy, giz, 0)) + 1e-6:
                continue

            closed_set.add(current)

            # 检查是否到达目标空间位置
            if current[0] == gix and current[1] == giy and current[2] == giz:
                found = True
                break

            for next_state, cost in self._get_neighbors(current):
                if next_state in closed_set:
                    continue
                tentative_g = g_score[current] + cost
                if next_state not in g_score or tentative_g < g_score[next_state]:
                    came_from[next_state] = current
                    g_score[next_state] = tentative_g
                    h = self._heuristic(next_state, (gix, giy, giz, 0))
                    f = tentative_g + h
                    counter += 1
                    heapq.heappush(open_set, (f, counter, next_state))

        planning_time_ms = (time.time() - t0) * 1000

        if not found:
            return PlannedRoute(
                route_id=route_id,
                task_id=task_id,
                aircraft_id=aircraft_id,
                status="failed",
                message=f"未找到可行路径 (迭代 {iterations} 次)",
                algorithm="SpaceTimeAStar",
                planning_time_ms=planning_time_ms,
            )

        # 回溯路径
        path_states = []
        while current in came_from:
            path_states.append(current)
            current = came_from[current]
        path_states.append(start_state)
        path_states.reverse()

        # 转换为连续坐标轨迹
        trajectory = []
        for ix, iy, iz, it in path_states:
            x, y, z = self.config.to_world(ix, iy, iz)
            t = self.config.step_to_time(it)
            trajectory.append(TrajectoryPoint(x=x, y=y, z=z, t=t))

        # 计算航向角（调度模式 extras=False 时跳过以节省 ~3%）
        if extras:
            for i in range(1, len(trajectory)):
                p0 = trajectory[i - 1]
                p1 = trajectory[i]
                dx = p1.x - p0.x
                dy = p1.y - p0.y
                if abs(dx) > 1e-6 or abs(dy) > 1e-6:
                    trajectory[i].heading = float(np.degrees(np.arctan2(dy, dx)))
                else:
                    trajectory[i].heading = trajectory[i - 1].heading

        route = PlannedRoute(
            route_id=route_id,
            task_id=task_id,
            aircraft_id=aircraft_id,
            trajectory=trajectory,
            status="planned",
            algorithm="SpaceTimeAStar",
            planning_time_ms=planning_time_ms,
        )
        route.compute_metrics()

        # 计算障碍物距离风险（调度模式 extras=False 时跳过，节省 ~8%）
        if extras:
            self._compute_risk(route)

        return route

    def _compute_risk(self, route: PlannedRoute):
        """计算航线的风险指标（距障碍物最近距离）"""
        if self.obstacle_set is None or not route.trajectory:
            return

        min_dist = float("inf")
        risks = []
        for p in route.trajectory:
            point_min_dist = float("inf")
            # 距静态障碍物最近距离（包围盒距离）
            for obs in self.obstacle_set.static_obstacles:
                dx = max(abs(p.x - obs.x) - obs.width, 0.0)
                dy = max(abs(p.y - obs.y) - obs.depth, 0.0)
                dz = max(max(p.z - (obs.z + obs.height), 0.0), max(obs.z - p.z, 0.0))
                dist = np.sqrt(dx * dx + dy * dy + dz * dz)
                point_min_dist = min(point_min_dist, dist)

            # 距动态障碍物最近距离
            for obs in self.obstacle_set.dynamic_obstacles:
                pos = obs.position_at(p.t)
                if pos is not None:
                    dx = p.x - pos[0]
                    dy = p.y - pos[1]
                    dz = p.z - pos[2]
                    dist = np.sqrt(dx * dx + dy * dy + dz * dz) - obs.safety_radius
                    point_min_dist = min(point_min_dist, dist)

            min_dist = min(min_dist, point_min_dist)
            # 风险值钳制在 [0, 1]
            risk = np.clip(1.0 - point_min_dist / 50.0, 0.0, 1.0) if point_min_dist != float("inf") else 0.0
            risks.append(risk)

        route.min_obstacle_distance = max(0.0, min_dist) if min_dist != float("inf") else 999.0
        route.max_risk = max(risks) if risks else 0.0
        route.avg_risk = float(np.mean(risks)) if risks else 0.0
