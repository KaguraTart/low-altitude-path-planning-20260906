"""
飞行器性能与任务约束建模 (3.26 模块二)

定义无人机/飞行器的物理性能约束（最大速度、最大爬升率、最大转弯率、
续航等），以及任务约束（航点、时间窗、载荷等）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class AircraftPerformance:
    """飞行器性能参数"""

    aircraft_id: str
    model: str = "generic-quadcopter"

    # 运动学约束
    max_speed: float = 15.0        # 最大水平速度 (m/s)
    max_climb_rate: float = 5.0    # 最大爬升率 (m/s)
    max_descent_rate: float = 3.0  # 最大下降率 (m/s)
    max_turn_rate: float = 45.0    # 最大转弯率 (度/秒)
    max_acceleration: float = 3.0  # 最大加速度 (m/s^2)

    # 几何约束
    rotor_radius: float = 0.5      # 旋翼半径 (m)
    safety_margin: float = 5.0     # 安全距离裕度 (m)

    # 续航约束
    max_flight_time: float = 1800.0  # 最大飞行时间 (秒) = 30分钟
    max_range: float = 15000.0       # 最大航程 (米)

    # 运行高度限制
    min_altitude: float = 10.0     # 最低飞行高度 (m)
    max_altitude: float = 300.0    # 最高飞行高度 (m)

    def is_speed_valid(self, vx: float, vy: float, vz: float) -> bool:
        """检查速度是否在性能范围内"""
        horizontal_speed = np.sqrt(vx * vx + vy * vy)
        if horizontal_speed > self.max_speed + 1e-6:
            return False
        if vz > self.max_climb_rate + 1e-6:
            return False
        if vz < -self.max_descent_rate - 1e-6:
            return False
        return True

    def is_altitude_valid(self, z: float) -> bool:
        """检查高度是否在运行范围内"""
        return self.min_altitude <= z <= self.max_altitude

    def estimate_energy(self, distance: float, altitude_change: float) -> float:
        """
        估算能耗（相对值，用于多目标优化）
        简化模型：能耗 = 水平距离 * k1 + 爬升高度 * k2 + 下降高度 * k3
        """
        k_horizontal = 1.0
        k_climb = 3.0
        k_descent = 0.5
        climb = max(0, altitude_change)
        descent = max(0, -altitude_change)
        return distance * k_horizontal + climb * k_climb + descent * k_descent


@dataclass
class Waypoint:
    """航点"""

    wp_id: str
    x: float
    y: float
    z: float
    # 到达时间窗 (秒)，None 表示无约束
    earliest_arrival: Optional[float] = None
    latest_arrival: Optional[float] = None
    # 停留时间 (秒)
    hover_time: float = 0.0
    # 航点类型: "start", "end", "via", "task"
    wp_type: str = "via"
    # 任务描述（如巡检、拍照、投放）
    task_description: str = ""


@dataclass
class FlightTask:
    """飞行任务定义"""

    task_id: str
    task_name: str
    aircraft_id: str

    # 航点序列（含起点和终点）
    waypoints: List[Waypoint] = field(default_factory=list)

    # 任务优先级 (1-10, 10最高)
    priority: int = 5

    # 任务类型: "delivery", "inspection", "patrol", "surveillance", "other"
    task_type: str = "other"

    # 期望起飞时间 (秒)
    departure_time: float = 0.0

    # 多目标优化权重
    objective_weights: dict = field(
        default_factory=lambda: {
            "distance": 0.3,
            "time": 0.3,
            "risk": 0.2,
            "energy": 0.2,
        }
    )

    @property
    def start(self) -> Waypoint:
        return self.waypoints[0]

    @property
    def end(self) -> Waypoint:
        return self.waypoints[-1]

    def validate(self) -> Tuple[bool, str]:
        """验证任务定义是否合法"""
        if len(self.waypoints) < 2:
            return False, "至少需要起点和终点两个航点"
        if not (1 <= self.priority <= 10):
            return False, "优先级必须在 1-10 之间"
        total_weight = sum(self.objective_weights.values())
        if abs(total_weight - 1.0) > 1e-6:
            return False, f"目标权重之和必须为 1.0，当前为 {total_weight}"
        return True, "valid"
