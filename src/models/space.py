"""
多维时空约束建模 (3.25 模块一)

定义三维空间 + 时间维度的四维时空网格模型，
支持空间分辨率、时间分辨率配置，以及空域边界、禁飞区等约束。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Tuple

import numpy as np


@dataclass
class SpaceTimeConfig:
    """时空网格配置参数"""

    # 空间范围 (米)
    x_min: float = 0.0
    x_max: float = 5000.0
    y_min: float = 0.0
    y_max: float = 5000.0
    z_min: float = 0.0
    z_max: float = 500.0

    # 空间分辨率 (米/格)
    dx: float = 50.0
    dy: float = 50.0
    dz: float = 25.0

    # 时间范围 (秒)
    t_min: float = 0.0
    t_max: float = 600.0

    # 时间分辨率 (秒/步)
    dt: float = 5.0

    @property
    def nx(self) -> int:
        """X 方向网格数"""
        return int(np.ceil((self.x_max - self.x_min) / self.dx))

    @property
    def ny(self) -> int:
        """Y 方向网格数"""
        return int(np.ceil((self.y_max - self.y_min) / self.dy))

    @property
    def nz(self) -> int:
        """Z 方向网格数"""
        return int(np.ceil((self.z_max - self.z_min) / self.dz))

    @property
    def nt(self) -> int:
        """时间步数"""
        return int(np.ceil((self.t_max - self.t_min) / self.dt))

    def to_grid(self, x: float, y: float, z: float) -> Tuple[int, int, int]:
        """连续坐标转网格索引"""
        ix = int(np.clip((x - self.x_min) / self.dx, 0, self.nx - 1))
        iy = int(np.clip((y - self.y_min) / self.dy, 0, self.ny - 1))
        iz = int(np.clip((z - self.z_min) / self.dz, 0, self.nz - 1))
        return ix, iy, iz

    def to_world(self, ix: int, iy: int, iz: int) -> Tuple[float, float, float]:
        """网格索引转连续坐标（网格中心）"""
        x = self.x_min + (ix + 0.5) * self.dx
        y = self.y_min + (iy + 0.5) * self.dy
        z = self.z_min + (iz + 0.5) * self.dz
        return x, y, z

    def time_to_step(self, t: float) -> int:
        """时间转时间步索引"""
        return int(np.clip((t - self.t_min) / self.dt, 0, self.nt - 1))

    def step_to_time(self, step: int) -> float:
        """时间步转时间（步中心）"""
        return self.t_min + (step + 0.5) * self.dt


@dataclass
class NoFlyZone:
    """禁飞区 / 空域约束（静态空间约束）"""

    zone_id: str
    name: str
    # 圆柱形禁飞区
    center_x: float
    center_y: float
    radius: float
    z_min: float = 0.0
    z_max: float = 500.0
    # 约束类型: "forbidden" 禁止进入, "restricted" 限制高度
    zone_type: str = "forbidden"

    def contains(self, x: float, y: float, z: float) -> bool:
        """判断点是否在禁飞区内"""
        if z < self.z_min or z > self.z_max:
            return False
        dx = x - self.center_x
        dy = y - self.center_y
        return (dx * dx + dy * dy) <= self.radius * self.radius


@dataclass
class AirspaceBoundary:
    """空域边界约束"""

    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float

    def contains(self, x: float, y: float, z: float) -> bool:
        return (
            self.x_min <= x <= self.x_max
            and self.y_min <= y <= self.y_max
            and self.z_min <= z <= self.z_max
        )


@dataclass
class SpaceTimeGrid:
    """
    四维时空占用网格 (x, y, z, t)

    occupancy[ix, iy, iz, it] = True 表示该时空格被占用（有障碍）
    """

    config: SpaceTimeConfig
    occupancy: np.ndarray = field(init=False)

    def __post_init__(self):
        self.occupancy = np.zeros(
            (self.config.nx, self.config.ny, self.config.nz, self.config.nt),
            dtype=bool,
        )

    def set_static_obstacle(self, x: float, y: float, z: float, radius: float = 0.0):
        """设置静态球形障碍物（在所有时间步均占用）"""
        ix, iy, iz = self.config.to_grid(x, y, z)
        r_cells = max(1, int(np.ceil(radius / min(self.config.dx, self.config.dy))))
        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                for dz in range(-r_cells, r_cells + 1):
                    nx_, ny_, nz_ = ix + dx, iy + dy, iz + dz
                    if 0 <= nx_ < self.config.nx and 0 <= ny_ < self.config.ny and 0 <= nz_ < self.config.nz:
                        self.occupancy[nx_, ny_, nz_, :] = True

    def set_static_box(
        self,
        x_min: float, x_max: float,
        y_min: float, y_max: float,
        z_min: float, z_max: float,
    ):
        """
        设置静态长方体障碍物（精确包围盒）

        仅标记实际在障碍物包围盒内的网格，
        不会错误标记障碍物上方的空域。
        """
        ix_min, iy_min, iz_min = self.config.to_grid(x_min, y_min, z_min)
        ix_max, iy_max, iz_max = self.config.to_grid(x_max, y_max, z_max)
        # 确保范围有效
        ix_min = max(0, ix_min)
        iy_min = max(0, iy_min)
        iz_min = max(0, iz_min)
        ix_max = min(self.config.nx - 1, ix_max)
        iy_max = min(self.config.ny - 1, iy_max)
        iz_max = min(self.config.nz - 1, iz_max)
        self.occupancy[ix_min:ix_max + 1, iy_min:iy_max + 1, iz_min:iz_max + 1, :] = True

    def set_dynamic_obstacle(
        self,
        trajectory: list,  # list of (x, y, z, t)
        radius: float = 0.0,
    ):
        """设置动态障碍物（仅在特定时间步占用）"""
        r_cells = max(1, int(np.ceil(radius / min(self.config.dx, self.config.dy))))
        for x, y, z, t in trajectory:
            ix, iy, iz = self.config.to_grid(x, y, z)
            it = self.config.time_to_step(t)
            for dx in range(-r_cells, r_cells + 1):
                for dy in range(-r_cells, r_cells + 1):
                    for dz in range(-r_cells, r_cells + 1):
                        nx_, ny_, nz_ = ix + dx, iy + dy, iz + dz
                        if (
                            0 <= nx_ < self.config.nx
                            and 0 <= ny_ < self.config.ny
                            and 0 <= nz_ < self.config.nz
                        ):
                            self.occupancy[nx_, ny_, nz_, it] = True

    def is_free(self, ix: int, iy: int, iz: int, it: int) -> bool:
        """检查时空格是否空闲"""
        if (
            ix < 0 or ix >= self.config.nx
            or iy < 0 or iy >= self.config.ny
            or iz < 0 or iz >= self.config.nz
            or it < 0 or it >= self.config.nt
        ):
            return False
        return not self.occupancy[ix, iy, iz, it]

    def occupancy_count(self) -> int:
        """统计被占用的时空格总数"""
        return int(np.sum(self.occupancy))
