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

    def set_cell_cost(self, ix: int, iy: int, iz: int, it: int, cost: float):
        """设置单个时空格的通行代价"""
        if (
            0 <= ix < self.config.nx
            and 0 <= iy < self.config.ny
            and 0 <= iz < self.config.nz
            and 0 <= it < self.config.nt
        ):
            self.cost[ix, iy, iz, it] = float(cost)

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

    def get_cost(self, ix: int, iy: int, iz: int, it: int) -> float:
        """获取单格通行代价（越界返回 inf）"""
        if (
            0 <= ix < self.config.nx
            and 0 <= iy < self.config.ny
            and 0 <= iz < self.config.nz
            and 0 <= it < self.config.nt
        ):
            return float(self.cost[ix, iy, iz, it])
        return float("inf")

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

    def reset(self):
        """重置为默认代价"""
        self.cost.fill(self.default_cost)


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