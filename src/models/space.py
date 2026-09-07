"""
多维时空网格配置与占用网格 (3.25 模块一)

任务书 §三 总体要求：
"统一基于三维网格空间模型开发，网格作为搜索、避障和约束判断基础。"

本文件提供：
- SpaceTimeConfig: 时空网格配置参数（空间范围/分辨率 + 时间范围/步长）
- NoFlyZone: 禁飞区（圆柱体表达）
- AirspaceBoundary: 空域边界
- SpaceTimeGrid: 四维占用网格 occupancy[ix, iy, iz, it] = True 表示被占用

四维网格设计：
- 状态空间 (ix, iy, iz, it) ∈ N^4
- 坐标转换：to_grid(x, y, z) / to_world(ix, iy, iz)
- 时间转换：time_to_step(t) / step_to_time(it)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Tuple

import numpy as np


@dataclass
class SpaceTimeConfig:
    """时空网格配置参数

    空间范围 = [x_min, x_max] × [y_min, y_max] × [z_min, z_max]（米）
    时间范围 = [t_min, t_max]（秒）

    网格分辨率：
    - 空间：dx × dy × dz 米/格
    - 时间：dt 秒/步

    网格大小由 ceil((x_max-x_min)/dx) × ... 计算得到。
    """

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
        """X 方向网格数（自动计算）"""
        return int(np.ceil((self.x_max - self.x_min) / self.dx))

    @property
    def ny(self) -> int:
        """Y 方向网格数（自动计算）"""
        return int(np.ceil((self.y_max - self.y_min) / self.dy))

    @property
    def nz(self) -> int:
        """Z 方向网格数（自动计算）"""
        return int(np.ceil((self.z_max - self.z_min) / self.dz))

    @property
    def nt(self) -> int:
        """时间步数（自动计算）"""
        return int(np.ceil((self.t_max - self.t_min) / self.dt))

    def to_grid(self, x: float, y: float, z: float) -> Tuple[int, int, int]:
        """连续坐标 (x, y, z) → 网格索引 (ix, iy, iz)

        使用 np.clip 钳制到合法范围（越界返回边界格）。
        任务书 P1 改进点：应改为抛错而非静默钳制。
        """
        ix = int(np.clip((x - self.x_min) / self.dx, 0, self.nx - 1))
        iy = int(np.clip((y - self.y_min) / self.dy, 0, self.ny - 1))
        iz = int(np.clip((z - self.z_min) / self.dz, 0, self.nz - 1))
        return ix, iy, iz

    def to_world(self, ix: int, iy: int, iz: int) -> Tuple[float, float, float]:
        """网格索引 (ix, iy, iz) → 连续坐标 (x, y, z)（返回网格中心点）"""
        x = self.x_min + (ix + 0.5) * self.dx
        y = self.y_min + (iy + 0.5) * self.dy
        z = self.z_min + (iz + 0.5) * self.dz
        return x, y, z

    def time_to_step(self, t: float) -> int:
        """时间 t (秒) → 时间步索引 it

        使用 np.clip 钳制到合法范围（越界返回边界步）。
        """
        return int(np.clip((t - self.t_min) / self.dt, 0, self.nt - 1))

    def step_to_time(self, step: int) -> float:
        """时间步 it → 时间 t (秒)（返回步中心时刻）"""
        return self.t_min + (step + 0.5) * self.dt


@dataclass
class NoFlyZone:
    """禁飞区 / 空域约束（静态空间约束，圆柱体表达）

    用于表达机场净空区、军事禁区等"任何时刻都禁止飞行"的空间。
    """

    zone_id: str
    name: str
    # 圆柱体参数
    center_x: float
    center_y: float
    radius: float
    z_min: float = 0.0
    z_max: float = 500.0
    # 约束类型
    zone_type: str = "forbidden"  # forbidden=禁止 / restricted=限制高度

    def contains(self, x: float, y: float, z: float) -> bool:
        """判断点 (x, y, z) 是否在禁飞区内"""
        if z < self.z_min or z > self.z_max:
            return False
        dx = x - self.center_x
        dy = y - self.center_y
        return (dx * dx + dy * dy) <= self.radius * self.radius


@dataclass
class AirspaceBoundary:
    """空域边界约束（长方体表达）"""

    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float

    def contains(self, x: float, y: float, z: float) -> bool:
        """判断点是否在空域内"""
        return (
            self.x_min <= x <= self.x_max
            and self.y_min <= y <= self.y_max
            and self.z_min <= z <= self.z_max
        )


@dataclass
class SpaceTimeGrid:
    """四维时空占用网格 occupancy[ix, iy, iz, it] = True 表示被占用

    存储形状：(nx, ny, nz, nt) 的 bool 数组
    - True：被障碍物占用，A* 不能进入
    - False：可通过

    设计原则：
    - 静态障碍物在所有时间步均占用（set_static_box）
    - 动态障碍物仅在特定时间步占用（set_dynamic_obstacle，轨迹插值）
    - 占用表示"能不能走"，与通行代价（CostGrid）正交
    """

    config: SpaceTimeConfig
    occupancy: np.ndarray = field(init=False)

    def __post_init__(self):
        """初始化占用网格（全 False）"""
        self.occupancy = np.zeros(
            (self.config.nx, self.config.ny, self.config.nz, self.config.nt),
            dtype=bool,
        )

    def set_static_obstacle(self, x: float, y: float, z: float, radius: float = 0.0):
        """设置球形静态障碍物（所有时间步均占用）

        Args:
            x, y, z: 障碍物中心
            radius: 半径（米，0 表示单格）
        """
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
        """设置长方体静态障碍物（精确包围盒）

        仅标记实际在障碍物内的网格，不浪费上方空域。
        任务书 §五.1 要求"模拟建筑物、禁飞区"使用此方法。
        """
        ix_min, iy_min, iz_min = self.config.to_grid(x_min, y_min, z_min)
        ix_max, iy_max, iz_max = self.config.to_grid(x_max, y_max, z_max)
        ix_min = max(0, ix_min)
        iy_min = max(0, iy_min)
        iz_min = max(0, iz_min)
        ix_max = min(self.config.nx - 1, ix_max)
        iy_max = min(self.config.ny - 1, iy_max)
        iz_max = min(self.config.nz - 1, iz_max)
        self.occupancy[ix_min:ix_max + 1, iy_min:iy_max + 1, iz_min:iz_max + 1, :] = True

    def set_dynamic_obstacle(
        self,
        trajectory: list,
        radius: float = 0.0,
    ):
        """设置动态障碍物（仅在特定时间步占用）

        Args:
            trajectory: [(x, y, z, t), ...] 轨迹点列表
            radius: 安全半径（米，0 表示单格）
        """
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
        """检查时空格 (ix, iy, iz, it) 是否空闲（越界视为占用）"""
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