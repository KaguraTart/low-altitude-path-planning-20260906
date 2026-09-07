"""
轨迹点与规划结果模型 (3.26 模块五)

任务书 §十 输出要求：
- 规划状态 SUCCESS/FAILED
- 任务编号
- 航点序列
- 路径长度
- 预计时间
- 约束检查结果
- 规划解释
- 算法耗时

本文件提供：
- TrajectoryPoint: 单个时空轨迹点（位置 + 时间 + 速度 + 航向）
- PlannedRoute: 单条规划航线（含指标 + 状态 + 算法信息 + 解释）
- PlanningResult: 批量规划结果汇总
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class TrajectoryPoint:
    """单个轨迹点（带时间戳）"""

    x: float
    y: float
    z: float
    t: float
    # 速度分量（可选，运行时填充）
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    # 航向角（度，可选，由 A* 后处理填充）
    heading: float = 0.0

    def to_tuple(self) -> Tuple[float, float, float, float]:
        """转为 (x, y, z, t) 元组，便于调试"""
        return (self.x, self.y, self.z, self.t)


@dataclass
class PlanningExplanation:
    """规划结果的结构化解释（任务书 §六 解释模块）

    包含：
    - 综合得分（多目标加权）
    - 各目标的贡献度
    - 路径选择原因
    - 避障原因（穿越了哪些障碍）
    - 性能裕度（接近极限的程度）
    """

    summary: str = ""                          # 一句话总结
    score: float = 0.0                         # 综合得分（越低越优）
    objective_breakdown: dict = field(default_factory=dict)   # 每个目标的得分
    avoidance_count: int = 0                   # 避障次数（穿越了多少障碍）
    performance_margin: dict = field(default_factory=dict)   # 性能裕度（最大速度/爬升/转弯占比）
    notes: List[str] = field(default_factory=list)            # 补充说明

    def to_dict(self) -> dict:
        return {
            "summary": self.summary,
            "score": round(self.score, 4),
            "objective_breakdown": {k: round(v, 4) for k, v in self.objective_breakdown.items()},
            "avoidance_count": self.avoidance_count,
            "performance_margin": {k: round(v, 4) for k, v in self.performance_margin.items()},
            "notes": self.notes,
        }


@dataclass
class PlannedRoute:
    """单条规划航线（任务书 §十 输出要求）"""

    route_id: str
    task_id: str
    aircraft_id: str

    # 轨迹点序列
    trajectory: List[TrajectoryPoint] = field(default_factory=list)

    # 规划指标
    total_distance: float = 0.0       # 总航程 (米)
    total_time: float = 0.0           # 总飞行时间 (秒)
    total_energy: float = 0.0         # 估算总能耗
    max_risk: float = 0.0             # 最大风险值 (0-1)
    avg_risk: float = 0.0             # 平均风险值
    min_obstacle_distance: float = 0.0  # 距障碍物最近距离 (米)

    # 规划状态
    status: str = "planned"  # "planned", "failed", "partial"
    message: str = ""

    # 算法信息
    algorithm: str = ""
    planning_time_ms: float = 0.0

    # 规划解释（任务书 §六 解释模块）
    explanation: Optional[PlanningExplanation] = None

    def compute_metrics(self, performance=None):
        """根据轨迹计算各项指标

        Args:
            performance: 飞行器性能（用于计算能耗）
        """
        if len(self.trajectory) < 2:
            return

        total_dist = 0.0
        total_energy = 0.0
        for i in range(1, len(self.trajectory)):
            p0 = self.trajectory[i - 1]
            p1 = self.trajectory[i]
            dx = p1.x - p0.x
            dy = p1.y - p0.y
            dz = p1.z - p0.z
            seg_dist = np.sqrt(dx * dx + dy * dy + dz * dz)
            total_dist += seg_dist
            if performance is not None:
                total_energy += performance.estimate_energy(
                    np.sqrt(dx * dx + dy * dy), dz
                )

        self.total_distance = total_dist
        self.total_time = self.trajectory[-1].t - self.trajectory[0].t
        self.total_energy = total_energy if total_energy > 0 else total_dist * 1.0

    def to_dict(self) -> dict:
        """序列化为 dict（用于 JSON 输出）"""
        return {
            "route_id": self.route_id,
            "task_id": self.task_id,
            "aircraft_id": self.aircraft_id,
            "status": self.status,
            "message": self.message,
            "algorithm": self.algorithm,
            "planning_time_ms": round(self.planning_time_ms, 2),
            "metrics": {
                "total_distance_m": round(self.total_distance, 2),
                "total_time_s": round(self.total_time, 2),
                "total_energy": round(self.total_energy, 2),
                "max_risk": round(self.max_risk, 4),
                "avg_risk": round(self.avg_risk, 4),
                "min_obstacle_distance_m": round(self.min_obstacle_distance, 2),
            },
            "trajectory": [
                {
                    "x": round(p.x, 2),
                    "y": round(p.y, 2),
                    "z": round(p.z, 2),
                    "t": round(p.t, 2),
                    "heading": round(p.heading, 1),
                }
                for p in self.trajectory
            ],
            "explanation": self.explanation.to_dict() if self.explanation else None,
        }


@dataclass
class PlanningResult:
    """批量规划结果（多任务规划输出）"""

    request_id: str
    routes: List[PlannedRoute] = field(default_factory=list)
    total_planning_time_ms: float = 0.0
    success_count: int = 0
    failed_count: int = 0

    def summary(self) -> dict:
        """序列化为 dict（用于 JSON 输出）"""
        return {
            "request_id": self.request_id,
            "total_routes": len(self.routes),
            "success_count": self.success_count,
            "failed_count": self.failed_count,
            "total_planning_time_ms": round(self.total_planning_time_ms, 2),
            "routes": [r.to_dict() for r in self.routes],
        }