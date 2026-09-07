"""
飞行器性能与任务约束建模 (3.26 模块二)

定义无人机/飞行器的物理性能约束（重量、尺寸、最大速度、最大爬升率、
转弯率、续航等），以及任务约束（航点、时间窗、载荷等）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class PhysicalSpec:
    """飞行器物理参数（重量、尺寸）"""

    weight_kg: float = 1.0                  # 自重
    max_takeoff_weight_kg: float = 1.5      # 最大起飞重量
    payload_capacity_kg: float = 0.5        # 有效载荷
    wingspan_m: float = 0.5                 # 翼展
    length_m: float = 0.5                   # 长
    width_m: float = 0.5                    # 宽
    height_m: float = 0.2                   # 高
    rotor_radius_m: float = 0.2             # 旋翼半径
    diagonal_m: float = 0.7                 # 对角线


@dataclass
class PerformanceSpec:
    """飞行器运动性能（任务书 §五.3 飞行器性能数据）"""

    max_speed_mps: float = 15.0             # 最大水平速度 (m/s) — 必须满足
    max_climb_rate_mps: float = 5.0         # 最大爬升率 (m/s)
    max_descent_rate_mps: float = 3.0       # 最大下降率 (m/s)
    max_turn_rate_dps: float = 45.0         # 最大转弯率 (度/秒)
    max_acceleration_mps2: float = 3.0      # 最大加速度 (m/s²)
    max_deceleration_mps2: float = 2.5      # 最大减速度 (m/s²)
    min_turn_radius_m: float = 0.0          # 最小转弯半径（米），0 表示未指定；>0 时 A* 必须满足


@dataclass
class EnduranceSpec:
    """续航与电源"""

    max_flight_time_s: float = 1800.0       # 最大飞行时间
    max_range_m: float = 15000.0            # 最大航程
    battery_capacity_wh: float = 200.0      # 电池容量
    cruise_power_w: float = 500.0           # 巡航功率
    hover_power_w: float = 350.0            # 悬停功率


@dataclass
class OperationalSpec:
    """运行环境限制"""

    min_altitude_m: float = 10.0            # 最低飞行高度
    max_altitude_m: float = 300.0           # 最高飞行高度
    min_operating_temp_c: float = -10.0     # 最低工作温度
    max_operating_temp_c: float = 40.0      # 最高工作温度
    max_wind_resistance_mps: float = 12.0   # 最大抗风
    ip_rating: str = "IP43"                 # 防护等级


@dataclass
class SafetySpec:
    """安全参数"""

    rotor_radius_m: float = 0.2             # 旋翼半径
    safety_margin_m: float = 5.0            # 安全裕度
    geofence_compliant: bool = True         # 地理围栏合规
    return_to_home_altitude_m: float = 50.0  # 返航高度


@dataclass
class AircraftPerformance:
    """飞行器性能参数（兼容旧字段 + 扩展动力学字段）"""

    aircraft_id: str
    model: str = "generic-quadcopter"
    category: str = "general"             # heavy_payload / medium_payload / delivery / inspection
    manufacturer: str = "Unknown"

    # ===== 扩展字段（可选，向后兼容）=====
    physical: PhysicalSpec = field(default_factory=PhysicalSpec)
    performance: PerformanceSpec = field(default_factory=PerformanceSpec)
    endurance: EnduranceSpec = field(default_factory=EnduranceSpec)
    operational: OperationalSpec = field(default_factory=OperationalSpec)
    safety: SafetySpec = field(default_factory=SafetySpec)

    # ===== 兼容旧字段（保留以支持旧版 JSON 输入）=====
    max_speed: float = 15.0               # → performance.max_speed_mps
    max_climb_rate: float = 5.0
    max_descent_rate: float = 3.0
    max_turn_rate: float = 45.0
    max_acceleration: float = 3.0
    max_flight_time: float = 1800.0
    max_range: float = 15000.0
    min_altitude: float = 10.0
    max_altitude: float = 300.0
    rotor_radius: float = 0.5
    safety_margin: float = 5.0

    def is_speed_valid(self, vx: float, vy: float, vz: float) -> bool:
        """检查速度是否在性能范围内

        检查项：
        - 水平速度 ≤ max_speed_mps
        - 爬升率 ≤ max_climb_rate_mps
        - 下降率 ≤ max_descent_rate_mps

        注意：加速度约束通过相邻速度差检测（不在本函数中）。
        """
        horizontal_speed = np.sqrt(vx * vx + vy * vy)
        max_h = self.performance.max_speed_mps
        max_c = self.performance.max_climb_rate_mps
        max_d = self.performance.max_descent_rate_mps
        if horizontal_speed > max_h + 1e-6:
            return False
        if vz > max_c + 1e-6:
            return False
        if vz < -max_d - 1e-6:
            return False
        return True

    def is_acceleration_valid(self, ax: float, ay: float, az: float) -> bool:
        """检查加速度是否在性能范围内（用于相邻段速度差检查）"""
        a = np.sqrt(ax * ax + ay * ay + az * az)
        return a <= self.performance.max_acceleration_mps2 + 1e-6

    def is_turn_radius_valid(self, radius: float) -> bool:
        """检查转弯半径是否 ≥ 最小转弯半径（如果指定）"""
        min_r = self.performance.min_turn_radius_m
        if min_r <= 0:
            return True
        return radius >= min_r - 1e-6

    def is_altitude_valid(self, z: float) -> bool:
        """检查高度是否在运行范围内"""
        return self.operational.min_altitude_m <= z <= self.operational.max_altitude_m

    def estimate_energy(self, distance: float, altitude_change: float) -> float:
        """
        估算能耗（相对值，用于多目标优化）

        简化模型：能耗 = 水平距离 * k1 + 爬升高度 * k2 + 下降高度 * k3

        如果启用了 endurance 字段，使用 cruise_power 估算更真实的能耗（瓦时）。
        """
        k_horizontal = 1.0
        k_climb = 3.0
        k_descent = 0.5
        climb = max(0, altitude_change)
        descent = max(0, -altitude_change)

        # 基础估算
        base = distance * k_horizontal + climb * k_climb + descent * k_descent

        # 如果有续航数据，按真实功率估算（更接近真实能耗）
        if self.endurance.cruise_power_w > 0 and distance > 0:
            # 平均飞行时间 = 距离 / 巡航速度
            cruise_speed = self.performance.max_speed_mps * 0.7
            if cruise_speed > 0:
                time_h = distance / cruise_speed / 3600.0
                real_energy = self.endurance.cruise_power_w * time_h
                # 加权混合（保守）
                return max(base, real_energy * 0.001)

        return base

    def get_max_speed(self) -> float:
        """获取最大速度（优先 performance 字段）"""
        return self.performance.max_speed_mps or self.max_speed

    def get_max_climb_rate(self) -> float:
        return self.performance.max_climb_rate_mps or self.max_climb_rate

    def get_max_descent_rate(self) -> float:
        return self.performance.max_descent_rate_mps or self.max_descent_rate

    def get_max_flight_time(self) -> float:
        return self.endurance.max_flight_time_s or self.max_flight_time

    def get_max_range(self) -> float:
        return self.endurance.max_range_m or self.max_range

    def get_min_altitude(self) -> float:
        return self.operational.min_altitude_m or self.min_altitude

    def get_max_altitude(self) -> float:
        return self.operational.max_altitude_m or self.max_altitude

    def summary(self) -> dict:
        return {
            "aircraft_id": self.aircraft_id,
            "model": self.model,
            "category": self.category,
            "weight_kg": self.physical.weight_kg,
            "payload_kg": self.physical.payload_capacity_kg,
            "max_speed_mps": self.get_max_speed(),
            "max_range_m": self.get_max_range(),
            "max_flight_time_s": self.get_max_flight_time(),
            "battery_wh": self.endurance.battery_capacity_wh,
        }


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
class TaskRequirements:
    """任务需求约束"""

    payload_kg: float = 0.0                  # 需要的载荷重量
    required_endurance_s: float = 0.0        # 需要的最小续航
    required_range_m: float = 0.0            # 需要的最小航程
    required_max_speed_mps: float = 0.0      # 需要的最小最大速度
    min_ceiling_m: float = 0.0               # 需要的最低工作高度
    delivery_window: Optional[Tuple[float, float]] = None  # 投递时间窗 (start_s, end_s)
    priority_level: int = 1                   # 业务优先级 1-5 (5最高)
    service_type: str = "delivery"            # delivery / inspection / patrol / surveillance


@dataclass
class FlightTask:
    """飞行任务定义"""

    task_id: str
    task_name: str
    aircraft_id: str

    # 航点序列（含起点和终点）
    waypoints: List[Waypoint] = field(default_factory=list)

    # 任务优先级 (1-10, 10最高) — 调度优先级
    priority: int = 5

    # 任务类型
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

    # 任务需求约束（新增）
    requirements: TaskRequirements = field(default_factory=TaskRequirements)

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
            return False, "调度优先级必须在 1-10 之间"
        total_weight = sum(self.objective_weights.values())
        if abs(total_weight - 1.0) > 1e-6:
            return False, f"目标权重之和必须为 1.0，当前为 {total_weight}"
        if not (1 <= self.requirements.priority_level <= 5):
            return False, "业务优先级必须在 1-5 之间"
        return True, "valid"

    def check_aircraft_compatibility(self, perf: AircraftPerformance) -> Tuple[bool, List[str]]:
        """检查飞行器性能是否满足任务需求，返回 (是否兼容, 问题列表)"""
        problems = []
        req = self.requirements

        if req.payload_kg > perf.physical.payload_capacity_kg:
            problems.append(
                f"载荷不足: 需要 {req.payload_kg}kg, 可用 {perf.physical.payload_capacity_kg}kg"
            )
        if req.required_endurance_s > perf.get_max_flight_time():
            problems.append(
                f"续航不足: 需要 {req.required_endurance_s}s, 可用 {perf.get_max_flight_time()}s"
            )
        if req.required_max_speed_mps > perf.get_max_speed():
            problems.append(
                f"最大速度不足: 需要 {req.required_max_speed_mps}m/s, "
                f"可用 {perf.get_max_speed()}m/s"
            )
        if req.min_ceiling_m > perf.get_max_altitude():
            problems.append(
                f"飞行高度不足: 需要 {req.min_ceiling_m}m, 可用 {perf.get_max_altitude()}m"
            )

        return (len(problems) == 0, problems)