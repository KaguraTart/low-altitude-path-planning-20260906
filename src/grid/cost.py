"""
网格通行代价模块 (3.25 扩展)

在 SpaceTimeGrid 基础上，提供基于地形/障碍类型的通行代价字段。
不同位置有不同的"穿越难度"，A* 在搜索时会累加这个代价，
从而让规划器自然偏好低代价路径（如避开拥挤空域、贴近走廊等）。

设计要点：
- 与占用网格 (occupancy) 正交：占用表示"不能走"，cost 表示"能走但更费力"
- 代价矩阵形状与 occupancy 一致: (nx, ny, nz, nt)，dtype=float32
- 默认代价 1.0（普通通行），可通过 set_cell_cost 设置区域代价

应用场景：
- 城市 CBD：高层建筑周围代价 +50（绕飞更省能）
- 居民区：人口密集区代价 +20
- 河流/山区：直接标记 occupied（cost 无意义）
- 风场/恶劣气象：局部代价乘以 1.5-3.0
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from ..models.space import SpaceTimeGrid


class CostGrid:
    """网格通行代价层

    与 SpaceTimeGrid 同形状，每个时空格一个通行代价（默认 1.0）。
    累加代价用 float32 节省内存，N=10000 任务约 12 MB（比 occupancy 多 4 byte）。
    """

    def __init__(self, base_grid: SpaceTimeGrid, default_cost: float = 1.0):
        """
        Args:
            base_grid: 参考的时空占用网格（必须同形状）
            default_cost: 全局默认通行代价
        """
        self.base_grid = base_grid
        self.config = base_grid.config
        self.default_cost = float(default_cost)
        # 初始化为默认值（float32：精度足够 + 省内存）
        self.cost = np.full(
            (self.config.nx, self.config.ny, self.config.nz, self.config.nt),
            self.default_cost,
            dtype=np.float32,
        )
        # 全局最小代价，增量维护。规划器用它缩放启发函数，
        # 保证在任意代价倍率下 A* 启发函数仍然可采纳。
        self.min_cost = self.default_cost
        # 时不变代价的 3D 扁平视图 (nx, ny, nz)。
        # 净空代价层这类"整时间段一致"的配置是 A* 内循环最常见的形态，
        # 用 3D 视图查一次即可，避免每次都触碰 (nx,ny,nz,nt) 大数组的一个切片
        # 而反复跨越 nt=120 的 stride（缓存不友好，内循环里有 ~7 次/扩展）。
        # 一旦有人按具体时间步 set_cell_cost，就退化为按需访问 4D 数组。
        self._spatial: Optional[np.ndarray] = None
        # 是否仍为"时不变"代价（未被按具体时间步改写）
        self._time_invariant = True

    def _mark_time_variant(self):
        """标记出现随时间变化的代价，禁用 3D 扁平视图"""
        self._time_invariant = False
        self._spatial = None

    def _spatial_view(self) -> Optional[np.ndarray]:
        """时不变时返回 3D 视图，否则 None"""
        if self._spatial is None and self._time_invariant:
            # 沿时间轴取首个切片：set_box_cost / apply_wind_multiplier 对全部
            # 时间步写入同一值，因此首个切片即代表所有时间步。
            self._spatial = np.ascontiguousarray(self.cost[:, :, :, 0])
        return self._spatial

    def _track_min(self, region: np.ndarray):
        """按受影响区域增量更新 min_cost"""
        if region.size:
            region_min = float(region.min())
            if region_min < self.min_cost:
                self.min_cost = region_min

    def set_cell_cost(self, ix: int, iy: int, iz: int, it: int, cost: float):
        """设置单个时空格的通行代价"""
        if (
            0 <= ix < self.config.nx
            and 0 <= iy < self.config.ny
            and 0 <= iz < self.config.nz
            and 0 <= it < self.config.nt
        ):
            self.cost[ix, iy, iz, it] = float(cost)
            self.min_cost = min(self.min_cost, float(cost))
            # 只改单个时间步 → 全时间步不再一致，退回 4D 访问
            self._mark_time_variant()

    def set_box_cost(
        self,
        x_min: float, x_max: float,
        y_min: float, y_max: float,
        z_min: float, z_max: float,
        cost: float,
    ):
        """设置长方体区域内的通行代价"""
        ix_min, iy_min, iz_min = self.config.to_grid(x_min, y_min, z_min)
        ix_max, iy_max, iz_max = self.config.to_grid(x_max, y_max, z_max)
        ix_min = max(0, ix_min)
        iy_min = max(0, iy_min)
        iz_min = max(0, iz_min)
        ix_max = min(self.config.nx - 1, ix_max)
        iy_max = min(self.config.ny - 1, iy_max)
        iz_max = min(self.config.nz - 1, iz_max)
        self.cost[
            ix_min:ix_max + 1,
            iy_min:iy_max + 1,
            iz_min:iz_max + 1,
            :
        ] = float(cost)
        self._track_min(self.cost[
            ix_min:ix_max + 1, iy_min:iy_max + 1, iz_min:iz_max + 1, :
        ])

    def get_cost(self, ix: int, iy: int, iz: int, it: int) -> float:
        """获取单格通行代价（越界返回 inf）

        时不变代价走 3D 扁平视图（缓存友好），否则访问 4D 数组。
        边界判断在两条路径上都需要（3D 视图越界会抛 IndexError）。
        """
        if not (
            0 <= ix < self.config.nx
            and 0 <= iy < self.config.ny
            and 0 <= iz < self.config.nz
            and 0 <= it < self.config.nt
        ):
            return float("inf")
        spatial = self._spatial_view()
        if spatial is not None:
            return float(spatial[ix, iy, iz])
        return float(self.cost[ix, iy, iz, it])

    def lookup_array(self) -> np.ndarray:
        """返回可直接索引的代价数组，供搜索内循环热路径使用。

        时不变时返回 3D `(nx, ny, nz)` 视图，否则返回 4D `(nx, ny, nz, nt)`。
        调用方需自行保证索引合法（A\* 的邻域生成已在查代价前完成边界检查）。

        相比逐次调用 :meth:`get_cost`，这省掉了每次查询的 Python 函数调用开销——
        A\* 每次扩展要查 6 个空间邻居 + 1 个等待动作，函数调用成本在热循环里不可忽略。
        """
        spatial = self._spatial_view()
        return spatial if spatial is not None else self.cost

    def get_world_cost(self, x: float, y: float, z: float, t: float) -> float:
        """世界坐标 → 通行代价"""
        ix, iy, iz = self.config.to_grid(x, y, z)
        it = self.config.time_to_step(t)
        return self.get_cost(ix, iy, iz, it)

    def apply_wind_multiplier(
        self,
        x_min: float, x_max: float,
        y_min: float, y_max: float,
        z_min: float, z_max: float,
        multiplier: float,
    ):
        """对区域施加乘性代价（如气象/风场影响：cost *= multiplier）"""
        ix_min, iy_min, iz_min = self.config.to_grid(x_min, y_min, z_min)
        ix_max, iy_max, iz_max = self.config.to_grid(x_max, y_max, z_max)
        ix_min = max(0, ix_min)
        iy_min = max(0, iy_min)
        iz_min = max(0, iz_min)
        ix_max = min(self.config.nx - 1, ix_max)
        iy_max = min(self.config.ny - 1, iy_max)
        iz_max = min(self.config.nz - 1, iz_max)
        self.cost[
            ix_min:ix_max + 1,
            iy_min:iy_max + 1,
            iz_min:iz_max + 1,
            :
        ] *= float(multiplier)
        self._track_min(self.cost[
            ix_min:ix_max + 1, iy_min:iy_max + 1, iz_min:iz_max + 1, :
        ])

    def reset(self):
        """重置为默认代价"""
        self.cost.fill(self.default_cost)
        self.min_cost = self.default_cost
        # 重置后所有时间步一致，恢复时不变状态并丢弃旧视图
        self._time_invariant = True
        self._spatial = None


def build_default_cost_grid(
    base_grid: SpaceTimeGrid,
    cbd_box: Optional[tuple] = None,
    cbd_cost: float = 10.0,
    residential_box: Optional[tuple] = None,
    residential_cost: float = 5.0,
) -> CostGrid:
    """
    构建默认的城市代价网格（在 occupancy 基础上叠加区域代价）

    Args:
        base_grid: 占用网格
        cbd_box: (x_min, x_max, y_min, y_max, z_min, z_max) 中央商务区
        cbd_cost: CBD 内通行代价（默认 10.0，绕飞成本高）
        residential_box: 居民区盒子
        residential_cost: 居民区代价（默认 5.0）
    """
    cg = CostGrid(base_grid, default_cost=1.0)
    if cbd_box is not None:
        cg.set_box_cost(*cbd_box, cost=cbd_cost)
    if residential_box is not None:
        cg.set_box_cost(*residential_box, cost=residential_cost)
    return cg


def build_clearance_cost_grid(
    base_grid: SpaceTimeGrid,
    obstacle_set,
    clearance: float = 200.0,
    clearance_cost: float = 3.0,
    skip_types: tuple = ("terrain",),
) -> CostGrid:
    """
    按障碍物净空距离构建代价网格（数据驱动，无需人工划定区域）

    思路：把每个静态障碍物的包围盒向外膨胀 `clearance` 米，该膨胀区内
    通行代价抬高到 `clearance_cost`。规划器因此天然偏好远离建筑群的航路，
    与 3.26 的 risk 目标（距障碍物越远风险越低）方向一致。

    障碍物本体已在 occupancy 中标记为不可通行，所以这里只标注"缓冲带"，
    两个图层正交叠加。

    Args:
        base_grid: 占用网格
        obstacle_set: ObstacleSet 实例
        clearance: 障碍物净空膨胀半径（米）
        clearance_cost: 膨胀区的通行代价倍率（默认 3.0）
        skip_types: 跳过的障碍物类型（如地形本身通常不需要净空惩罚）

    Returns:
        CostGrid: 代价层（min_cost 仍为 1.0，因为只抬高不放低）
    """
    cg = CostGrid(base_grid, default_cost=1.0)
    for obs in getattr(obstacle_set, "static_obstacles", []):
        if obs.obstacle_type in skip_types:
            continue
        cg.set_box_cost(
            x_min=obs.x - obs.width - clearance,
            x_max=obs.x + obs.width + clearance,
            y_min=obs.y - obs.depth - clearance,
            y_max=obs.y + obs.depth + clearance,
            z_min=obs.z - clearance,
            z_max=obs.z + obs.height + clearance,
            cost=clearance_cost,
        )
    return cg