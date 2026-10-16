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

from ..models.space import SpaceTimeConfig, segment_closest_approach
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
        self._route_cells: Dict[str, set] = {}
        self._segments: Dict[str, tuple] = {}
        self._segment_index: Dict[Cell, set] = {}

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
        if route.route_id in self._route_cells:
            self.unmark_route(route)
        points = [(p.x, p.y, p.z, p.t) for p in route.trajectory]
        segments = list(zip(points, points[1:])) or [(p, p) for p in points]
        cells = set()
        self._segments[route.route_id] = (route.aircraft_id, segments)
        for index, (start, end) in enumerate(segments):
            for cell in self.config.segment_cells(start, end):
                self._segment_index.setdefault(cell, set()).add((route.route_id, index))
            cells.update(self.config.segment_cells(start, end, radius=safety_radius))
        for cell in cells:
            self._add_cell(cell, route.aircraft_id, route.route_id)
        self._route_cells[route.route_id] = cells
        self._mark_radius[route.route_id] = safety_radius

    def unmark_route(self, route: PlannedRoute, safety_radius: float = 0.0):
        """Remove the exact registered snapshot, including swept segment cells."""
        self._mark_radius.pop(route.route_id, safety_radius)
        registered = self._segments.pop(route.route_id, None)
        aircraft_id = registered[0] if registered else route.aircraft_id
        for cell in self._route_cells.pop(route.route_id, set()):
            by_aircraft = self._occupancy.get(cell)
            if not by_aircraft:
                continue
            route_ids = by_aircraft.get(aircraft_id, set())
            route_ids.discard(route.route_id)
            if not route_ids:
                by_aircraft.pop(aircraft_id, None)
            if not by_aircraft:
                del self._occupancy[cell]
                if self._mask_valid and self._base_mask is not None:
                    self._base_mask[cell] = False
        if registered:
            for index, (start, end) in enumerate(registered[1]):
                for cell in self.config.segment_cells(start, end):
                    entries = self._segment_index.get(cell)
                    if entries is not None:
                        entries.discard((route.route_id, index))
                        if not entries:
                            del self._segment_index[cell]

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
        """Find continuous conflicts, retaining the configured time tolerance.

        Swept voxel indexing supplies a conservative broad phase. The narrow
        phase uses actual coordinates/timestamps rather than cell centers, and
        minimizes separation over complete segment pairs (including edge swaps).
        """
        conflicts = []
        if min_separation <= 0 or time_window < 0:
            return conflicts
        points = [(p.x, p.y, p.z, p.t) for p in route.trajectory]
        segments = list(zip(points, points[1:])) or [(p, p) for p in points]
        seen = set()
        for start, end in segments:
            candidates = set()
            for cell in self.config.segment_cells(start, end, radius=min_separation, time_padding=time_window):
                candidates.update(self._segment_index.get(cell, ()))
            for other_route_id, index in sorted(candidates):
                aircraft_id, other_segments = self._segments[other_route_id]
                if aircraft_id == route.aircraft_id:
                    continue
                other_start, other_end = other_segments[index]
                closest = segment_closest_approach(start, end, other_start, other_end, time_window)
                if closest is None or closest[0] >= min_separation:
                    continue
                distance, timestamp, position_a, position_b = closest
                key = (other_route_id, round(timestamp, 8))
                if key in seen:
                    continue
                seen.add(key)
                conflicts.append({
                    "route_a": route.route_id,
                    "route_b": other_route_id,
                    "aircraft_b": aircraft_id,
                    "time": round(timestamp, 2),
                    "position_a": tuple(round(v, 1) for v in position_a),
                    "position_b": tuple(round(v, 1) for v in position_b),
                    "distance": round(distance, 2),
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
            for cell in self._route_cells.get(rid, set()):
                by_aircraft = self._occupancy.get(cell, {})
                mask[cell] = any(route_ids - {rid} for route_ids in by_aircraft.values())

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
        self._route_cells.clear()
        self._segments.clear()
        self._segment_index.clear()
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
