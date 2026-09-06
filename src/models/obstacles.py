"""
障碍物数据模型 (3.25 模块二: 静态与动态避障)

定义静态障碍物（建筑物、地形、禁飞区）和动态障碍物（其他飞行器、
移动车辆等）的数据结构，支持多源数据融合。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class StaticObstacle:
    """静态障碍物（建筑物、地形凸起、固定设施等）"""

    obstacle_id: str
    name: str
    obstacle_type: str  # "building", "terrain", "tower", "other"
    # 位置 (米)
    x: float
    y: float
    z: float  # 障碍物顶部高度
    # 几何尺寸
    width: float = 20.0   # X 方向半宽
    depth: float = 20.0   # Y 方向半深
    height: float = 50.0  # 高度（从 z 基准到顶部）
    # 数据源标识
    source: str = "unknown"

    def contains(self, x: float, y: float, z: float) -> bool:
        """判断点是否在障碍物包围盒内"""
        return (
            abs(x - self.x) <= self.width
            and abs(y - self.y) <= self.depth
            and 0 <= z <= self.z + self.height
        )

    def to_cylinder_radius(self) -> float:
        """近似为圆柱时的等效半径"""
        return max(self.width, self.depth)


@dataclass
class DynamicObstacle:
    """动态障碍物（其他飞行器、移动目标等）"""

    obstacle_id: str
    name: str
    obstacle_type: str  # "uav", "vehicle", "bird_flock", "other"
    # 轨迹点列表: [(x, y, z, t), ...]
    trajectory: List[Tuple[float, float, float, float]] = field(default_factory=list)
    # 安全半径 (米)
    safety_radius: float = 10.0
    # 数据源标识
    source: str = "unknown"

    def position_at(self, t: float) -> Optional[Tuple[float, float, float]]:
        """线性插值获取 t 时刻的位置"""
        if not self.trajectory:
            return None
        if t <= self.trajectory[0][3]:
            return self.trajectory[0][:3]
        if t >= self.trajectory[-1][3]:
            return self.trajectory[-1][:3]
        for i in range(len(self.trajectory) - 1):
            t0 = self.trajectory[i][3]
            t1 = self.trajectory[i + 1][3]
            if t0 <= t <= t1:
                alpha = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
                x = self.trajectory[i][0] + alpha * (self.trajectory[i + 1][0] - self.trajectory[i][0])
                y = self.trajectory[i][1] + alpha * (self.trajectory[i + 1][1] - self.trajectory[i][1])
                z = self.trajectory[i][2] + alpha * (self.trajectory[i + 1][2] - self.trajectory[i][2])
                return (x, y, z)
        return None

    def collides(self, x: float, y: float, z: float, t: float) -> bool:
        """判断在 t 时刻，点 (x,y,z) 是否与该动态障碍碰撞"""
        pos = self.position_at(t)
        if pos is None:
            return False
        dx = x - pos[0]
        dy = y - pos[1]
        dz = z - pos[2]
        dist = np.sqrt(dx * dx + dy * dy + dz * dz)
        return dist <= self.safety_radius


@dataclass
class ObstacleSet:
    """障碍物集合（静态 + 动态），支持多源数据融合"""

    static_obstacles: List[StaticObstacle] = field(default_factory=list)
    dynamic_obstacles: List[DynamicObstacle] = field(default_factory=list)

    def add_static(self, obs: StaticObstacle):
        self.static_obstacles.append(obs)

    def add_dynamic(self, obs: DynamicObstacle):
        self.dynamic_obstacles.append(obs)

    def is_static_blocked(self, x: float, y: float, z: float) -> bool:
        """检查点是否被静态障碍物阻挡"""
        for obs in self.static_obstacles:
            if obs.contains(x, y, z):
                return True
        return False

    def is_dynamic_blocked(self, x: float, y: float, z: float, t: float) -> bool:
        """检查点在 t 时刻是否被动态障碍物阻挡"""
        for obs in self.dynamic_obstacles:
            if obs.collides(x, y, z, t):
                return True
        return False

    def is_blocked(self, x: float, y: float, z: float, t: float) -> bool:
        """检查点在 t 时刻是否被任何障碍物阻挡"""
        return self.is_static_blocked(x, y, z) or self.is_dynamic_blocked(x, y, z, t)

    def summary(self) -> dict:
        return {
            "static_count": len(self.static_obstacles),
            "dynamic_count": len(self.dynamic_obstacles),
            "static_types": list(set(o.obstacle_type for o in self.static_obstacles)),
            "dynamic_types": list(set(o.obstacle_type for o in self.dynamic_obstacles)),
        }
