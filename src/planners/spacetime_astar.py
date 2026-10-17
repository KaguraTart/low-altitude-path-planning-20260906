"""
多维时空 A* 路径规划算法 (3.25 核心算法 / 任务书 §八 推荐技术路线)

任务书 §五.1：
"建立空间+时间联合搜索模型。
 节点表示(x,y,z,t)四维状态。
 支持任务时间窗约束。
 支持动态占用状态更新。"

任务书 §五.3：
"静态障碍避障：搜索过程中禁止进入障碍网格。生成绕障路径。计算路径长度和避障代价。"

本算法特点：
- 四维状态空间 (ix, iy, iz, it)
- 6/26 邻域空间移动 + 等待动作（每次推进 1 个时间步）
- 启发函数：6 邻域曼哈顿 / 26 邻域切比雪夫距离 × dt（可采纳）
- 代价函数：时间步长 + 微小距离惩罚（打破平局）
- Closed set 避免重复扩展
- Stale entry 跳过过期条目

接口：
- plan(start, goal, start_time, ...): 异步执行 A* 搜索，返回 PlannedRoute
"""

from __future__ import annotations

import heapq
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

from ..models.space import SpaceTimeConfig, SpaceTimeGrid, segment_box_interval, segment_closest_approach
from ..models.obstacles import ObstacleSet
from ..models.task import TrajectoryPoint, PlannedRoute
from ..grid.cost import CostGrid


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
        max_iterations: Optional[int] = None,
        grid: Optional[SpaceTimeGrid] = None,
        cost_grid: Optional[CostGrid] = None,
        min_altitude: Optional[float] = None,
        max_altitude: Optional[float] = None,
        max_climb_rate: Optional[float] = None,
        max_descent_rate: Optional[float] = None,
        occupancy_mask: Optional[np.ndarray] = None,
    ):
        self.config = config
        self.obstacle_set = obstacle_set
        self.max_speed = max_speed
        self.wait_enabled = wait_enabled
        self.neighbor_mode = neighbor_mode
        self.max_iterations = max_iterations
        # 可选的通行代价层（3.25 扩展）：让规划器偏好低代价空域。
        # 与 occupancy 正交——occupancy 决定"不能走"，cost 决定"更费力"。
        self.cost_grid = cost_grid
        # 飞行器高度包线（operational min/max altitude）。
        # 与网格 z 范围求交集后作为搜索的 z 边界，避免规划出超出升限的航路。
        self.min_altitude = min_altitude
        self.max_altitude = max_altitude
        self.max_climb_rate = max_climb_rate
        self.max_descent_rate = max_descent_rate
        # 多机冲突【避免】：其它已规划航线的禁飞时空掩码（与 SpaceTimeGrid
        # 同构的 bool 数组）。为 None 时退化为单机规划，行为与原来完全一致。
        # 与 obstacle 障碍网格是「并集」关系：两者都必须为空闲才能进入。
        self.occupancy_mask = occupancy_mask
        self._iz_min, self._iz_max = self._resolve_altitude_band()

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

    def _resolve_altitude_band(self) -> Tuple[int, int]:
        """飞行器高度包线 ∩ 网格 z 范围 → (iz_min, iz_max)"""
        low = max(self.config.z_min, self.min_altitude if self.min_altitude is not None else self.config.z_min)
        high = min(self.config.z_max, self.max_altitude if self.max_altitude is not None else self.config.z_max)
        # Limits apply to actual centers, not to the containing voxel index.
        iz_min = max(0, int(np.ceil((low - self.config.z_min) / self.config.dz - 0.5 - 1e-10)))
        iz_max = min(self.config.nz - 1, int(np.floor((high - self.config.z_min) / self.config.dz - 0.5 + 1e-10)))
        return iz_min, iz_max

    def _connector_steps(self, start: Tuple[float, float, float], end: Tuple[float, float, float]) -> int:
        delta = np.asarray(end) - np.asarray(start)
        if np.all(delta == 0):
            return 0
        if self.max_speed <= 0:
            return self.config.nt
        duration = float(np.linalg.norm(delta)) / self.max_speed
        rate = self.max_climb_rate if delta[2] > 0 else self.max_descent_rate
        if delta[2] != 0 and rate is not None:
            if rate <= 0:
                return self.config.nt
            duration = max(duration, abs(float(delta[2])) / rate)
        return max(1, int(np.ceil(duration / self.config.dt - 1e-10)))

    def _edge_is_free(self, start: Tuple[float, float, float, float], end: Tuple[float, float, float, float]) -> bool:
        if not self.grid.segment_is_free(start, end, self.occupancy_mask):
            return False
        if self.obstacle_set is None:
            return True
        for obs in self.obstacle_set.static_obstacles:
            if segment_box_interval(start[:3], end[:3],
                    (obs.x - obs.width, obs.y - obs.depth, obs.z),
                    (obs.x + obs.width, obs.y + obs.depth, obs.z + obs.height)) is not None:
                return False
        for obs in self.obstacle_set.dynamic_obstacles:
            points = obs.trajectory
            for a, b in list(zip(points, points[1:])) or [(p, p) for p in points]:
                if start[3] > b[3] or a[3] > end[3]:
                    continue
                if any(max(start[axis], end[axis]) + obs.safety_radius < min(a[axis], b[axis])
                       or max(a[axis], b[axis]) + obs.safety_radius < min(start[axis], end[axis])
                       for axis in range(3)):
                    continue
                closest = segment_closest_approach(start, end, a, b)
                if closest is not None and closest[0] <= obs.safety_radius:
                    return False
        return True

    def altitude_error(self, point: Tuple[float, float, float], label: str) -> Optional[str]:
        """检查一个空间点是否落在飞行器高度包线内，返回错误描述或 None"""
        if self.min_altitude is not None and point[2] < self.min_altitude - 1e-6:
            return f"{label}高度 {point[2]:.1f}m 低于飞行器最低高度 {self.min_altitude}m"
        if self.max_altitude is not None and point[2] > self.max_altitude + 1e-6:
            return f"{label}高度 {point[2]:.1f}m 超过飞行器最高高度 {self.max_altitude}m"
        return None

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
        启发函数: 邻域移动步数下界 * 时间步长 * 全局最小代价倍率

        无代价层时 min_cost = 1.0，使用标准空间移动时间下界。
        启用代价层时每条边的真实代价 ≥ dt * min_cost，按此缩放后
        启发函数仍然是真实剩余代价的下界，保证最优性不被破坏。
        """
        ix, iy, iz, it = state
        gx, gy, gz, gt = goal
        distances = (abs(ix - gx), abs(iy - gy), abs(iz - gz))
        moves = max(distances) if self.neighbor_mode == "26" else sum(distances)
        minimum = self._min_cost_scale
        estimate = moves * self.config.dt * minimum
        if moves and self.cost_grid is not None:
            # Every arrival pays at least one goal-cell step. Accounting for
            # this unavoidable cost prevents an expensive destination from
            # causing exhaustive expansion over every earlier time layer.
            costs = self.cost_grid.lookup_array()
            goal_cost = float(costs[gx, gy, gz]) if costs.ndim == 3 else float(np.min(costs[gx, gy, gz, :]))
            estimate += self.config.dt * max(0.0, goal_cost - minimum)
        return estimate

    @property
    def _min_cost_scale(self) -> float:
        """代价层带来的启发函数缩放系数"""
        if self.cost_grid is None:
            return 1.0
        return max(1e-6, self.cost_grid.min_cost)

    def _get_neighbors(
        self, state: Tuple[int, int, int, int]
    ) -> List[Tuple[Tuple[int, int, int, int], float]]:
        """
        获取当前状态的所有可行后继状态及代价
        返回: [(next_state, cost), ...]
        """
        ix, iy, iz, it = state
        neighbors = []

        # 代价数组直取，避免在热循环里反复调用 CostGrid.get_cost
        # （每次扩展要查 6 个空间邻居 + 1 个等待动作）
        cmul = self.cost_grid.lookup_array() if self.cost_grid is not None else None
        cmul_3d = cmul is not None and cmul.ndim == 3
        # 多机禁飞掩码（同理：热循环里只做一次数组取用）
        occ = self.occupancy_mask

        # 空间移动（6/26 邻域）
        for dx, dy, dz in self.neighbors:
            nix, niy, niz = ix + dx, iy + dy, iz + dz
            nit = it + 1  # 每次移动推进一个时间步

            # 边界检查（含飞行器高度包线）
            if not (
                0 <= nix < self.config.nx
                and 0 <= niy < self.config.ny
                and self._iz_min <= niz <= self._iz_max
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

            # 爬升/下降率约束：网格 dz 通常远大于"单时间步可爬升的高度"，
            # 因此一个 dz 位移可能需要多个时间步（阶梯式爬升/下降），
            # 否则爬升率受限的飞行器将完全无法改变高度。
            steps = 1
            if dz != 0:
                dz_m = abs(dz) * self.config.dz
                rate_limit = self.max_climb_rate if dz > 0 else self.max_descent_rate
                if rate_limit is not None and rate_limit <= 0:
                    continue
                if rate_limit is not None:
                    steps = max(1, int(np.ceil(dz_m / (rate_limit * self.config.dt) - 1e-9)))
                nit = it + steps
                if nit >= self.config.nt:
                    continue
                # 阶梯沿途也必须无碰撞（同一竖直线上的中间格）
                blocked = False
                for k in range(1, steps):
                    mid_z = iz + int(round(dz * k / steps))
                    if not self.grid.is_free(nix, niy, mid_z, it + k):
                        blocked = True
                        break
                    if occ is not None and occ[nix, niy, mid_z, it + k]:
                        blocked = True
                        break
                if blocked:
                    continue

            # 碰撞检查（环境障碍 ∪ 其它飞行器占用）
            if not self.grid.is_free(nix, niy, niz, nit):
                continue
            if occ is not None and occ[nix, niy, niz, nit]:
                continue

            # 代价 = 时间步长（每次移动消耗 dt 时间）+ 微小距离惩罚（打破平局，偏好短路径）
            cost = self.config.dt * steps + spatial_dist * 0.001
            if cmul is not None:
                # 通行代价倍率（默认 1.0；净空缓冲带等区域可 > 1）
                cost *= cmul[nix, niy, niz] if cmul_3d else cmul[nix, niy, niz, nit]
            neighbors.append(((nix, niy, niz, nit), cost))

        # 等待动作（原地停留，时间推进）
        if self.wait_enabled:
            nit = it + 1
            if (
                nit < self.config.nt
                and self.grid.is_free(ix, iy, iz, nit)
                and not (occ is not None and occ[ix, iy, iz, nit])
            ):
                cost = self.config.dt * 1.05  # 等待略贵于移动，鼓励前进
                if cmul is not None:
                    cost *= cmul[ix, iy, iz] if cmul_3d else cmul[ix, iy, iz, nit]
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

        def failure(message: str) -> PlannedRoute:
            return PlannedRoute(route_id=route_id, task_id=task_id, aircraft_id=aircraft_id,
                                status="failed", message=message, algorithm="SpaceTimeAStar",
                                planning_time_ms=(time.time() - t0) * 1000)

        for label, point in (("起点", start), ("终点", goal)):
            if not (np.isfinite(point).all()
                    and self.config.x_min <= point[0] <= self.config.x_max
                    and self.config.y_min <= point[1] <= self.config.y_max
                    and self.config.z_min <= point[2] <= self.config.z_max):
                return failure(f"{label}超出可用空域")
        if not np.isfinite(start_time) or not self.config.t_min <= start_time < self.config.t_max:
            return failure("起飞时间超出可用时间范围")
        if self._iz_min > self._iz_max:
            return failure("飞行器高度包线内没有可用网格中心")

        # 起点/终点高度可行性预检（在占用检查之前，给出可读的原因）
        for label, point in (("起点", start), ("终点", goal)):
            err = self.altitude_error(point, label)
            if err is not None:
                return PlannedRoute(
                    route_id=route_id,
                    task_id=task_id,
                    aircraft_id=aircraft_id,
                    status="failed",
                    message=err,
                    algorithm="SpaceTimeAStar",
                    planning_time_ms=(time.time() - t0) * 1000,
                )

        # 转换为网格坐标
        six, siy, siz = self.config.to_grid(*start)
        gix, giy, giz = self.config.to_grid(*goal)
        siz = min(max(siz, self._iz_min), self._iz_max)
        giz = min(max(giz, self._iz_min), self._iz_max)
        departure_step = self.config.time_to_step(start_time)
        actual_start_cell = (*self.config.to_grid(*start), departure_step)
        if not self.grid.is_free(*actual_start_cell):
            return failure("起点被障碍物占用")
        if self.occupancy_mask is not None and self.occupancy_mask[actual_start_cell]:
            return failure(f"起点在 t={start_time:.0f}s 被其它飞行器占用（多机冲突，建议延迟起飞）")
        time_offset = start_time - self.config.step_to_time(departure_step)
        start_anchor = self.config.to_world(six, siy, siz)
        goal_anchor = self.config.to_world(gix, giy, giz)
        entry_steps = self._connector_steps(start, start_anchor)
        exit_steps = self._connector_steps(goal_anchor, goal)
        sit = departure_step + entry_steps
        anchor_time = start_time + entry_steps * self.config.dt
        if sit >= self.config.nt or not self._edge_is_free((*start, start_time), (*start_anchor, anchor_time)):
            return failure("无法安全连接精确起点与网格中心")
        if exit_steps >= self.config.nt:
            return failure("无法在时间范围内连接精确终点")
        # A permanently blocked connector cannot be repaired by waiting. Detect
        # it before spending the entire search budget on unreachable goal times.
        for ix, iy, iz, _ in self.config.segment_cells((*goal_anchor, start_time),
                (*goal, start_time + exit_steps * self.config.dt)):
            if bool(self.grid.occupancy[ix, iy, iz, :].all()):
                return failure("精确终点连接段被障碍物永久占用")
        if self.obstacle_set is not None:
            for obs in self.obstacle_set.static_obstacles:
                if segment_box_interval(goal_anchor, goal,
                        (obs.x - obs.width, obs.y - obs.depth, obs.z),
                        (obs.x + obs.width, obs.y + obs.depth, obs.z + obs.height)) is not None:
                    return failure("精确终点连接段与静态障碍物相交")

        start_state = (six, siy, siz, sit)
        goal_state = (gix, giy, giz, -1)  # 时间步不固定，到达空间即可

        def remaining_steps(state: Tuple[int, int, int, int]) -> int:
            distances = (abs(state[0] - gix), abs(state[1] - giy), abs(state[2] - giz))
            return (max(distances) if self.neighbor_mode == "26" else sum(distances)) + exit_steps

        if anchor_time + remaining_steps(start_state) * self.config.dt >= self.config.t_max:
            return failure("剩余时间不足以到达精确终点")

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
        # 起点在其它飞行器的占用格里（多机冲突）——
        # 这种情况搜索无解（第一步就出不去），提前给出可读原因，
        # 让 ParallelPlanner.avoid_conflicts 能改用「延迟起飞」重试。
        if (
            self.occupancy_mask is not None
            and self.occupancy_mask[six, siy, siz, sit]
        ):
            return PlannedRoute(
                route_id=route_id,
                task_id=task_id,
                aircraft_id=aircraft_id,
                status="failed",
                message=(
                    f"起点在 t={sit * self.config.dt:.0f}s 被其它飞行器占用"
                    "（多机冲突，建议延迟起飞）"
                ),
                algorithm="SpaceTimeAStar",
                planning_time_ms=(time.time() - t0) * 1000,
            )

        # 终点若被其它飞行器在整个时间轴上占满，搜索必然失败——
        # 提前判定，避免白烧 max_iterations 次迭代（万级任务下是主要的尾延迟来源）。
        if self.occupancy_mask is not None and 0 <= gix < self.config.nx and 0 <= giy < self.config.ny and 0 <= giz < self.config.nz:
            if bool(self.occupancy_mask[gix, giy, giz, :].all()):
                return PlannedRoute(
                    route_id=route_id,
                    task_id=task_id,
                    aircraft_id=aircraft_id,
                    status="failed",
                    message="终点在整个时间轴上被其它飞行器占用（多机冲突，无法抵达）",
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

        while open_set and (max_iterations is None or iterations < max_iterations):
            iterations += 1
            f_current, _, current = heapq.heappop(open_set)

            # 跳过过期条目（该状态已被更优路径扩展过）
            if current in closed_set:
                continue
            if current in g_score and f_current > g_score[current] + self._heuristic(current, (gix, giy, giz, 0)) + 1e-6:
                continue

            closed_set.add(current)
            if self.config.step_to_time(current[3]) + time_offset + remaining_steps(current) * self.config.dt >= self.config.t_max:
                continue

            # 检查是否到达目标空间位置
            if current[0] == gix and current[1] == giy and current[2] == giz:
                arrival_time = self.config.step_to_time(current[3]) + time_offset
                final_time = arrival_time + exit_steps * self.config.dt
                if self._edge_is_free((*goal_anchor, arrival_time), (*goal, final_time)):
                    found = True
                    break

            for next_state, cost in self._get_neighbors(current):
                if next_state in closed_set:
                    continue
                edge_start = (*self.config.to_world(*current[:3]), self.config.step_to_time(current[3]) + time_offset)
                edge_end = (*self.config.to_world(*next_state[:3]), self.config.step_to_time(next_state[3]) + time_offset)
                if not self._edge_is_free(edge_start, edge_end):
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
        if entry_steps:
            trajectory.append(TrajectoryPoint(x=start[0], y=start[1], z=start[2], t=start_time))
        for ix, iy, iz, it in path_states:
            x, y, z = self.config.to_world(ix, iy, iz)
            t = self.config.step_to_time(it) + time_offset
            trajectory.append(TrajectoryPoint(x=x, y=y, z=z, t=t))
        trajectory[0].x, trajectory[0].y, trajectory[0].z = start
        trajectory[0].t = start_time
        if exit_steps:
            trajectory.append(TrajectoryPoint(x=goal[0], y=goal[1], z=goal[2],
                                              t=trajectory[-1].t + exit_steps * self.config.dt))
        trajectory[-1].x, trajectory[-1].y, trajectory[-1].z = goal
        # A stationary task still needs an explicit, checked dwell interval.
        if len(trajectory) == 1:
            point = trajectory[0]
            if not self._edge_is_free((point.x, point.y, point.z, point.t),
                                      (point.x, point.y, point.z, point.t + self.config.dt)):
                return failure("原地等待时间内存在占用")
            trajectory.append(TrajectoryPoint(x=point.x, y=point.y, z=point.z, t=point.t + self.config.dt))

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
