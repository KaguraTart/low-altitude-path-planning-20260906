"""
时空占用表与冲突协调器（任务书 §三.2 时空资源管理 / §五.4 多机航线冲突避免）

独立成模块的原因：
- `SpaceTimeAStar` 需要消费占用表（多机冲突**避免**，而不只是事后检测），
  而 `parallel_planner` 又依赖 A*，放在一起会造成循环导入。
- 占用表本身是纯数据结构，没有规划逻辑，单独放置更清晰。

提供：
- SpatioTemporalOccupancyTable: 记录已规划航线的 (ix, iy, iz, it) 占用
- ConflictResolver: 冲突协调建议（延迟起飞 / 高度调整）
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import numpy as np

from ..models.space import SpaceTimeConfig
from ..models.task import PlannedRoute

Cell = Tuple[int, int, int, int]


class SpatioTemporalOccupancyTable:
    """
    时空占用表（任务书 §三.2 时空资源管理 + §四 步骤4）

    记录所有已规划航线的 (x, y, z, t) 占用情况，供后续规划参考和冲突检测。

    实现细节：
    - 用 dict 存储离散化的占用格 (ix, iy, iz, it)
    - 每个格记录"被哪些飞机、哪些航线占用"，支持冲突溯源

    API：
    - mark_route(route): 标记整条航线占用
    - unmark_route(route): 清除航线占用
    - is_occupied(x, y, z, t, exclude_aircraft): 查询占用（可排除自身）
    - find_conflicts(route, min_separation): 找出与新航线的所有冲突
    - to_dense_mask(): 生成 4D bool 掩码，供 A* 作为"禁飞时空格"直接索引
    """

    def __init__(self, config: SpaceTimeConfig, min_separation: float = 10.0):
        self.config = config
        self.min_separation = min_separation
        # {(ix, iy, iz, it): {aircraft_id: {route_id, ...}}} 占用记录
        self._occupancy: Dict[Cell, Dict[str, set]] = {}
        # 稠密底图的增量缓存。_occupancy 的"某格是否有任意航线"与
        # _base_mask 保持同步，这样 to_dense_mask() 每次只需 O(1) 取
        # 底图 + O(|待排除航线|) 修正，而不是 O(|occupancy|) 全表扫描
        # （万级航线下全表扫描是 O(n²·m²)，曾让 N=10000 压测跑数小时）。
        self._base_mask: Optional[np.ndarray] = None
        self._mask_valid = False
        # route_id → mark_route 时用的半径，排除自身占用时要按同一半径还原
        self._mark_radius: Dict[str, float] = {}

    # ------------------------------------------------------------------
    # 占用登记
    # ------------------------------------------------------------------
    def _world_to_grid_with_radius(
        self, x: float, y: float, z: float, t: float, radius: float
    ) -> List[Cell]:
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

    def _add_cell(self, cell: Cell, aircraft_id: str, route_id: str) -> None:
        by_aircraft = self._occupancy.get(cell)
        if by_aircraft is None:
            by_aircraft = {}
            self._occupancy[cell] = by_aircraft
            if self._mask_valid and self._base_mask is not None:
                self._base_mask[cell] = True
        by_aircraft.setdefault(aircraft_id, set()).add(route_id)

    def mark_route(self, route: PlannedRoute, safety_radius: float = 0.0):
        """将整条航线的占用格标记为 route.aircraft_id

        Args:
            route: 已规划航线
            safety_radius: 占用半径（米，飞机周围的额外安全圈）。
                注意：大半径会让占用表体积膨胀（每点 27 格），
                万级航线请保持 0，安全间隔改由 to_dense_mask 的
                dilation_cells 控制（按轴计算，只需 floor 层）。
        """
        for p in route.trajectory:
            for cell in self._world_to_grid_with_radius(p.x, p.y, p.z, p.t, safety_radius):
                self._add_cell(cell, route.aircraft_id, route.route_id)
        self._mark_radius[route.route_id] = safety_radius

    def unmark_route(self, route: PlannedRoute, safety_radius: float = 0.0):
        """从占用表中移除航线的占用格"""
        radius = self._mark_radius.pop(route.route_id, safety_radius)
        for p in route.trajectory:
            for cell in self._world_to_grid_with_radius(p.x, p.y, p.z, p.t, radius):
                by_aircraft = self._occupancy.get(cell)
                if by_aircraft is None:
                    continue
                routes_of_aircraft = by_aircraft.get(route.aircraft_id)
                if routes_of_aircraft is not None:
                    routes_of_aircraft.discard(route.route_id)
                    if not routes_of_aircraft:
                        del by_aircraft[route.aircraft_id]
                if not by_aircraft:
                    del self._occupancy[cell]
                    if self._mask_valid and self._base_mask is not None:
                        self._base_mask[cell] = False

    # ------------------------------------------------------------------
    # 查询
    # ------------------------------------------------------------------
    def is_occupied(
        self, x: float, y: float, z: float, t: float,
        exclude_aircraft: Optional[str] = None,
        safety_radius: float = 0.0,
    ) -> bool:
        """查询 (x, y, z, t) 是否被占用（可排除自身飞机）"""
        for cell in self._world_to_grid_with_radius(x, y, z, t, safety_radius):
            by_aircraft = self._occupancy.get(cell)
            if by_aircraft is None:
                continue
            if exclude_aircraft is None:
                if by_aircraft:
                    return True
            elif any(a != exclude_aircraft for a in by_aircraft):
                return True
        return False

    def find_conflicts(
        self, route: PlannedRoute, min_separation: float = 10.0,
        time_window: float = 5.0,
    ) -> List[dict]:
        """找出新航线与已登记航线之间的所有冲突

        复杂度 O(|trajectory| × 时间窗步数 × (2r+1)³)，其中
        r = floor(min_separation / 最小格长) + 1。
        旧实现对每个轨迹点遍历**全表**（O(|occupancy|)），在万级航线下
        是 O(n²·m²)；这里按"中心距 < min_separation 的格必落在 r 邻域内"
        只查局部，与全表扫描结果一致。

        Returns:
            [{"route_a", "route_b", "aircraft_b", "time", "position_a",
              "position_b", "distance"}, ...]
        """
        conflicts = []
        cfg = self.config
        nt, dt = cfg.nt, cfg.dt
        min_cell = min(cfg.dx, cfg.dy, cfg.dz)
        # 空间邻域半径（+1 余量：轨迹点经 to_grid 取整，可能偏离格心半格）
        r = int(min_separation // min_cell) + 1 if min_separation > 0 else 0
        rng = range(-r, r + 1)

        for pa in route.trajectory:
            ix, iy, iz = cfg.to_grid(pa.x, pa.y, pa.z)
            if dt > 0:
                # 时间窗覆盖到的步。约定（见 SpaceTimeConfig）：
                #   time_to_step(t) = int((t - t_min) / dt)   ← 向下取整
                #   step_to_time(it) = t_min + (it + 0.5) * dt ← 步**中心**
                # 由 |pa.t - step_to_time(it)| <= time_window 反解出 it 的闭区间，
                # 再向外扩到整数（floor/ceil），保证这里拿到的是**超集**，
                # 最后由下面的显式复核精确裁剪。若直接用 ceil/floor
                # （漏掉 ±0.5dt），会漏掉紧邻时间窗边界的步（假阴性），
                # 曾经的写法还会反过来把窗外一步纳入（假阳性）。
                half = 0.5 * dt
                it_lo = int(np.floor((pa.t - time_window - cfg.t_min - half) / dt))
                it_hi = int(np.ceil((pa.t + time_window - cfg.t_min - half) / dt))
            else:
                it_lo = it_hi = cfg.time_to_step(pa.t)
            for it_a in range(max(0, it_lo), min(nt - 1, it_hi) + 1):
                # 显式复核：与 PathValidator 朴素路径的 abs(pa.t - pb.t) > tw 保持一致
                if abs(pa.t - cfg.step_to_time(it_a)) > time_window:
                    continue
                for ddx in rng:
                    jx = ix + ddx
                    if not (0 <= jx < cfg.nx):
                        continue
                    for ddy in rng:
                        jy = iy + ddy
                        if not (0 <= jy < cfg.ny):
                            continue
                        for ddz in rng:
                            jz = iz + ddz
                            if not (0 <= jz < cfg.nz):
                                continue
                            by_aircraft = self._occupancy.get((jx, jy, jz, it_a))
                            if not by_aircraft:
                                continue
                            wx, wy, wz = cfg.to_world(jx, jy, jz)
                            dx = pa.x - wx
                            dy = pa.y - wy
                            dz = pa.z - wz
                            d = float(np.sqrt(dx * dx + dy * dy + dz * dz))
                            if d >= min_separation:
                                continue
                            for aircraft_id, route_ids in by_aircraft.items():
                                if aircraft_id == route.aircraft_id:
                                    continue  # 自身不算冲突
                                for other_route_id in route_ids:
                                    conflicts.append({
                                        "route_a": route.route_id,
                                        "route_b": other_route_id,
                                        "aircraft_b": aircraft_id,
                                        "time": round(pa.t, 2),
                                        "position_a": (round(pa.x, 1), round(pa.y, 1), round(pa.z, 1)),
                                        "position_b": (round(wx, 1), round(wy, 1), round(wz, 1)),
                                        "distance": round(d, 2),
                                    })
        return conflicts

    def clearance_cells(self, min_separation: Optional[float] = None) -> Tuple[int, int, int]:
        """安全间隔换算成每轴需要膨胀的格数

        推导：A* 轨迹点位于格心。若 A* 停在某个未被膨胀标记的格，
        则它与被占用格在**至少一个轴**上的格距 > r，世界距离
        ≥ (r+1) * cell_size > min_separation（当 r = floor(min_sep/cell)）。
        因此 floor 是既安全又最紧的取值。

        Returns:
            (rx, ry, rz) 每轴膨胀格数
        """
        sep = self.min_separation if min_separation is None else min_separation
        if sep <= 0:
            return (0, 0, 0)
        rx = int(sep // self.config.dx)
        ry = int(sep // self.config.dy)
        rz = int(sep // self.config.dz)
        return (rx, ry, rz)

    def to_dense_mask(
        self,
        exclude_route: Optional[PlannedRoute] = None,
        dilate: bool = True,
    ) -> np.ndarray:
        """
        生成供 A* 直接索引的稠密禁飞掩码 occupancy_mask[ix, iy, iz, it]

        掩码按 (ix, iy, iz, nt) 与 SpaceTimeGrid 同构，A* 内循环里
        只需一次数组索引（与 CostGrid.lookup_array() 同样的优化思路）。

        复杂度：底图 `_base_mask` 与 `_occupancy` 增量同步，每次调用
        只需 O(1) 取底图 + O(|exclude_route|) 修正 + 一次数组拷贝。
        （旧实现每次全表扫描 `_occupancy`；万级航下单次就 ~5s，
          上百次重规划会把 N=10000 压测拖成小时级。）

        Args:
            exclude_route: 要重规划的那条航线——它自己的占用不计入，
                否则起点永远被自己挡住。只重算该航线自己的格子，
                其余航线的占用原样保留（其 mark_route 时用的半径
                自动生效）。
            dilate: 是否按安全间隔膨胀（floor 层数，见 clearance_cells）

        Returns:
            bool 掩码，True = 该时空格禁飞
        """
        mask = self._ensure_base_mask().copy()

        if exclude_route is not None:
            rid = exclude_route.route_id
            radius = self._mark_radius.get(rid, 0.0)
            for p in exclude_route.trajectory:
                for cell in self._world_to_grid_with_radius(p.x, p.y, p.z, p.t, radius):
                    by_aircraft = self._occupancy.get(cell)
                    others = False
                    if by_aircraft:
                        for _aircraft_id, route_ids in by_aircraft.items():
                            if route_ids - {rid}:
                                others = True
                                break
                    mask[cell] = others

        if dilate:
            rx, ry, rz = self.clearance_cells()
            if rx or ry or rz:
                mask = _dilate_4d(mask, rx, ry, rz)
        return mask

    def _ensure_base_mask(self) -> np.ndarray:
        """底图：首次全量构建，之后由 _add_cell/unmark_route 增量维护"""
        if self._base_mask is None or not self._mask_valid:
            nx, ny, nz, nt = self.config.nx, self.config.ny, self.config.nz, self.config.nt
            m = np.zeros((nx, ny, nz, nt), dtype=bool)
            for ix, iy, iz, it in self._occupancy:
                m[ix, iy, iz, it] = True
            self._base_mask = m
            self._mask_valid = True
        return self._base_mask

    def clear(self):
        """清空占用表"""
        self._occupancy.clear()
        self._mark_radius.clear()
        self._base_mask = None
        self._mask_valid = False

    def __len__(self) -> int:
        return len(self._occupancy)


def _dilate_4d(mask: np.ndarray, rx: int, ry: int, rz: int) -> np.ndarray:
    """在 x/y/z 三轴上做 max(半径) 膨胀（时间轴不膨胀）

    用移位 OR 实现：每个方向一次切片赋值，比逐格 Python 循环快两个量级。
    """
    nx, ny, nz, nt = mask.shape
    out = mask.copy()
    for sx in range(-rx, rx + 1):
        for sy in range(-ry, ry + 1):
            for sz in range(-rz, rz + 1):
                if sx == 0 and sy == 0 and sz == 0:
                    continue
                dst_x = slice(max(0, -sx), nx - max(0, sx))
                src_x = slice(max(0, sx), nx - max(0, -sx))
                dst_y = slice(max(0, -sy), ny - max(0, sy))
                src_y = slice(max(0, sy), ny - max(0, -sy))
                dst_z = slice(max(0, -sz), nz - max(0, sz))
                src_z = slice(max(0, sz), nz - max(0, -sz))
                out[dst_x, dst_y, dst_z, :] |= mask[src_x, src_y, src_z, :]
    return out


class ConflictResolver:
    """
    冲突协调器（任务书 §三.2 冲突协调）

    发现冲突后触发重规划：
    - 策略 A：延迟起飞时间（等占用方飞过）
    - 策略 B：高度调整（+1 层）
    - 策略 C：完全重规划（带禁用格）——见 ParallelPlanner.avoid_conflicts
    """

    def __init__(self, config: SpaceTimeConfig, min_separation: float = 10.0):
        self.config = config
        self.min_separation = min_separation

    def suggest_delay(
        self,
        conflict: dict,
        max_delay_s: float = 60.0,
        other_route: Optional[PlannedRoute] = None,
    ) -> Optional[float]:
        """估算本方需要延迟多少秒才能避开对方

        有 other_route 时做**真实计算**：找出对方轨迹中"占用冲突点邻域"
        的时间区间 [t_in, t_out]，返回刚好晚于 t_out 的延迟量。
        没有时退化为保守估计（冲突时刻 + 一个时间窗），并仍受 max_delay_s 截断。

        Args:
            conflict: 冲突条目（含 time / position_b）
            max_delay_s: 可接受的最大延迟（秒）
            other_route: 对方航线（可选，提供则计算更准确）
        """
        t_c = float(conflict.get("time", 0.0))
        if other_route is not None and other_route.trajectory:
            pos_b = conflict.get("position_b")
            if pos_b is not None:
                bx, by, bz = pos_b
                # 对方通过冲突点附近的时间区间
                times = [
                    p.t for p in other_route.trajectory
                    if (p.x - bx) ** 2 + (p.y - by) ** 2 + (p.z - bz) ** 2
                    <= self.min_separation ** 2
                ]
                if times:
                    delay = (max(times) + self.config.dt) - t_c
                    return float(min(max(delay, 0.0), max_delay_s))
        # 保守估计：等一个时间窗再走
        fallback = t_c + self.config.dt
        return float(min(max(fallback, 0.0), max_delay_s))

    def suggest_altitude_shift(self, current_z: float, ceiling: Optional[float] = None) -> float:
        """建议调整到的高度（上移一个网格层）"""
        new_z = current_z + self.config.dz
        limit = ceiling if ceiling is not None else self.config.z_max
        if new_z <= limit:
            return new_z
        return current_z  # 已到顶，无法升
