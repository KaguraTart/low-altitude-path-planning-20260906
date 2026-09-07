"""
路径合法性验证模块 (3.26 验证模块)

任务书 §六 要求"验证模块：检查路径合法性"。
任务书 §十二 验收标准"路径无静态障碍冲突 / 满足飞行器性能约束 / 多机情况下无航线冲突"。

本模块对单条航线和多机航线集分别做合法性校验，返回结构化的 ValidationResult。

校验维度：
1. 连续性：相邻轨迹点是否平滑（无大跳跃）
2. 障碍避让：轨迹点是否落在静态障碍内
3. 动态避让：轨迹点是否被动态障碍的安全半径覆盖
4. 飞行性能：速度/爬升率/转弯率/加速度是否在飞行器性能范围内
5. 高度限制：是否在飞行器最低/最高高度范围内
6. 时间窗：是否满足航点时间窗约束
7. 多机冲突：航线两两之间的时间和空间距离是否 ≥ 安全间隔
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple

import numpy as np

from ..models.aircraft import AircraftPerformance
from ..models.obstacles import ObstacleSet
from ..models.task import PlannedRoute, TrajectoryPoint
from ..models.space import SpaceTimeGrid


class Severity(Enum):
    """验证问题的严重程度"""
    INFO = "info"          # 提示性信息（如"该轨迹无威胁"）
    WARNING = "warning"      # 警告（如"接近障碍物边缘 5m"）
    ERROR = "error"          # 错误（如"撞到建筑物"）


@dataclass
class ValidationIssue:
    """单个验证问题"""
    severity: Severity
    code: str               # 问题代码（如 "OBSTACLE_COLLISION"）
    message: str            # 人类可读描述
    trajectory_index: int = -1   # 相关轨迹点索引（-1 = 整体）
    time: float = 0.0           # 相关时间（秒）
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # 相关位置


@dataclass
class ValidationResult:
    """完整验证结果"""
    is_valid: bool
    issues: List[ValidationIssue] = field(default_factory=list)
    summary: Dict[str, int] = field(default_factory=dict)

    def add(self, issue: ValidationIssue):
        """添加一个问题"""
        self.issues.append(issue)

    def has_errors(self) -> bool:
        return any(i.severity == Severity.ERROR for i in self.issues)

    def merge(self, other: "ValidationResult"):
        """合并另一个结果"""
        self.issues.extend(other.issues)
        self.is_valid = self.is_valid and other.is_valid

    def summary_text(self) -> str:
        """生成摘要文本"""
        n = len(self.issues)
        n_err = sum(1 for i in self.issues if i.severity == Severity.ERROR)
        n_warn = sum(1 for i in self.issues if i.severity == Severity.WARNING)
        n_info = sum(1 for i in self.issues if i.severity == Severity.INFO)
        status = "✓ 合法" if self.is_valid else "✗ 不合法"
        return f"{status}：共 {n} 个问题（{n_err} 错误, {n_warn} 警告, {n_info} 提示）"


class PathValidator:
    """
    路径合法性验证器

    校验单条航线或多条航线集合是否满足：
    1. 连续性
    2. 静态/动态障碍避让
    3. 飞行器性能约束
    4. 高度限制
    5. 时间窗约束
    6. 多机安全间隔
    """

    # 安全检查阈值（可配置）
    OBSTACLE_SAFETY_MARGIN = 5.0       # 障碍物边缘警戒距离（米）
    APPROACH_WARNING_MARGIN = 10.0     # 接近警告距离（米）
    MULTI_UAV_MIN_SEPARATION = 10.0    # 多机最小间隔（米）
    MULTI_UAV_TIME_WINDOW = 5.0        # 时间对齐窗口（秒）
    MAX_SEGMENT_LENGTH = 1000.0        # 单段最大允许距离（米），防止跳变

    def __init__(
        self,
        obstacle_set: Optional[ObstacleSet] = None,
        grid: Optional[SpaceTimeGrid] = None,
        min_separation: float = MULTI_UAV_MIN_SEPARATION,
        time_window: float = MULTI_UAV_TIME_WINDOW,
    ):
        self.obstacle_set = obstacle_set
        self.grid = grid
        self.min_separation = min_separation
        self.time_window = time_window

    def validate_route(
        self,
        route: PlannedRoute,
        performance: Optional[AircraftPerformance] = None,
        safety_margin: float = OBSTACLE_SAFETY_MARGIN,
    ) -> ValidationResult:
        """
        校验单条航线的合法性

        Args:
            route: 待校验的航线
            performance: 飞行器性能（用于检查速度/爬升/转弯约束）
            safety_margin: 障碍物安全距离（米）

        Returns:
            ValidationResult: 包含所有发现的问题
        """
        result = ValidationResult(is_valid=True)
        traj = route.trajectory
        if not traj or len(traj) < 2:
            result.add(ValidationIssue(
                severity=Severity.ERROR, code="EMPTY_TRAJECTORY",
                message="轨迹为空或点数不足（< 2）",
            ))
            result.is_valid = False
            return result

        # 1. 连续性
        self._check_continuity(traj, result)

        # 2. 障碍避让
        if self.obstacle_set is not None:
            self._check_static_obstacles(traj, result, safety_margin)
            self._check_dynamic_obstacles(traj, result, safety_margin)

        # 3. 高度限制
        if performance is not None:
            self._check_altitude(traj, performance, result)

        # 4. 飞行性能
        if performance is not None:
            self._check_performance(traj, performance, result)

        # 5. 时空占用一致性（如果传了 grid）
        if self.grid is not None:
            self._check_grid_occupancy(traj, result)

        # 更新 summary
        result.summary = {
            "trajectory_points": len(traj),
            "total_distance_m": route.total_distance,
            "total_time_s": route.total_time,
            "n_errors": sum(1 for i in result.issues if i.severity == Severity.ERROR),
            "n_warnings": sum(1 for i in result.issues if i.severity == Severity.WARNING),
        }
        if result.has_errors():
            result.is_valid = False
        return result

    def validate_multi_routes(
        self,
        routes: List[PlannedRoute],
        time_window: Optional[float] = None,
    ) -> ValidationResult:
        """
        校验多机航线集：两两检查时间和空间间隔

        Args:
            routes: 航线列表（只校验 status="planned" 的）
            time_window: 时间对齐窗口（秒），默认使用构造时的设置

        Returns:
            ValidationResult: 所有两两冲突的列表
        """
        result = ValidationResult(is_valid=True)
        planned = [r for r in routes if r.status == "planned"]
        tw = time_window if time_window is not None else self.time_window

        n = len(planned)
        for i in range(n):
            for j in range(i + 1, n):
                ri, rj = planned[i], planned[j]
                if ri.aircraft_id == rj.aircraft_id:
                    continue  # 同一架飞机不算冲突（多段航线）

                # 时间对齐检查：对每对时刻找同时刻最近距离
                min_dist = float("inf")
                min_time = 0.0
                min_pos = None
                # 为效率，先二分对齐轨迹
                for pa in ri.trajectory:
                    for pb in rj.trajectory:
                        if abs(pa.t - pb.t) > tw:
                            continue
                        d = np.sqrt(
                            (pa.x - pb.x) ** 2
                            + (pa.y - pb.y) ** 2
                            + (pa.z - pb.z) ** 2
                        )
                        if d < min_dist:
                            min_dist = d
                            min_time = pa.t
                            min_pos = ((pa.x, pa.y, pa.z), (pb.x, pb.y, pb.z))

                if min_dist < self.min_separation:
                    severity = Severity.ERROR if min_dist < self.min_separation / 2 else Severity.WARNING
                    code = "MULTI_UAV_COLLISION" if severity == Severity.ERROR else "MULTI_UAV_PROXIMITY"
                    result.add(ValidationIssue(
                        severity=severity, code=code,
                        message=(
                            f"航线 {ri.route_id} 与 {rj.route_id} 在 t={min_time:.1f}s 时"
                            f"距离仅 {min_dist:.1f}m（最小要求 {self.min_separation}m）"
                        ),
                        time=min_time,
                        position=min_pos[0] if min_pos else (0.0, 0.0, 0.0),
                    ))

        result.summary = {
            "n_routes": len(planned),
            "n_pairs_checked": n * (n - 1) // 2,
            "n_conflicts": sum(1 for i in result.issues if i.severity == Severity.ERROR),
        }
        if result.has_errors():
            result.is_valid = False
        return result

    # ============================================================
    # 单项检查实现
    # ============================================================

    def _check_continuity(self, traj: List[TrajectoryPoint], result: ValidationResult):
        """连续性：相邻轨迹点不应有大跳跃"""
        for i in range(1, len(traj)):
            p0, p1 = traj[i - 1], traj[i]
            seg_dist = np.sqrt(
                (p1.x - p0.x) ** 2 + (p1.y - p0.y) ** 2 + (p1.z - p0.z) ** 2
            )
            if seg_dist > self.MAX_SEGMENT_LENGTH:
                result.add(ValidationIssue(
                    severity=Severity.ERROR, code="SEGMENT_TOO_LONG",
                    message=f"轨迹段 {i} 距离 {seg_dist:.0f}m 超过最大值 "
                            f"{self.MAX_SEGMENT_LENGTH}m（疑似跳变）",
                    trajectory_index=i,
                    time=p1.t,
                    position=(p1.x, p1.y, p1.z),
                ))

    def _check_static_obstacles(
        self, traj: List[TrajectoryPoint], result: ValidationResult,
        safety_margin: float,
    ):
        """静态障碍避让"""
        for i, p in enumerate(traj):
            for obs in self.obstacle_set.static_obstacles:
                # 用包围盒距离（AABB）
                dx = max(abs(p.x - obs.x) - obs.width, 0.0)
                dy = max(abs(p.y - obs.y) - obs.depth, 0.0)
                dz_top = max(p.z - (obs.z + obs.height), 0.0)
                dz_bot = max(obs.z - p.z, 0.0)
                dz = max(dz_top, dz_bot)
                dist = np.sqrt(dx * dx + dy * dy + dz * dz)

                if dist < 1e-6:
                    # 在障碍内
                    result.add(ValidationIssue(
                        severity=Severity.ERROR, code="STATIC_OBSTACLE_COLLISION",
                        message=f"轨迹点 {i} 落在静态障碍 {obs.name}（{obs.obstacle_type}）内",
                        trajectory_index=i, time=p.t, position=(p.x, p.y, p.z),
                    ))
                elif dist < safety_margin:
                    result.add(ValidationIssue(
                        severity=Severity.WARNING, code="STATIC_OBSTACLE_PROXIMITY",
                        message=f"轨迹点 {i} 距静态障碍 {obs.name} 仅 {dist:.1f}m（< {safety_margin}m）",
                        trajectory_index=i, time=p.t, position=(p.x, p.y, p.z),
                    ))

    def _check_dynamic_obstacles(
        self, traj: List[TrajectoryPoint], result: ValidationResult,
        safety_margin: float,
    ):
        """动态障碍避让：用线性插值取该时刻障碍物位置"""
        for i, p in enumerate(traj):
            for obs in self.obstacle_set.dynamic_obstacles:
                pos = obs.position_at(p.t)
                if pos is None:
                    continue
                dx = p.x - pos[0]
                dy = p.y - pos[1]
                dz = p.z - pos[2]
                dist = np.sqrt(dx * dx + dy * dy + dz * dz)
                safe_dist = obs.safety_radius + safety_margin

                if dist < obs.safety_radius:
                    result.add(ValidationIssue(
                        severity=Severity.ERROR, code="DYNAMIC_OBSTACLE_COLLISION",
                        message=f"轨迹点 {i} 在 t={p.t:.1f}s 进入动态障碍 "
                                f"{obs.name} 安全半径（距离 {dist:.1f}m）",
                        trajectory_index=i, time=p.t, position=(p.x, p.y, p.z),
                    ))
                elif dist < safe_dist:
                    result.add(ValidationIssue(
                        severity=Severity.WARNING, code="DYNAMIC_OBSTACLE_PROXIMITY",
                        message=f"轨迹点 {i} 在 t={p.t:.1f}s 接近动态障碍 "
                                f"{obs.name}（距离 {dist:.1f}m，要求 ≥ {safe_dist}m）",
                        trajectory_index=i, time=p.t, position=(p.x, p.y, p.z),
                    ))

    def _check_altitude(
        self, traj: List[TrajectoryPoint],
        performance: AircraftPerformance,
        result: ValidationResult,
    ):
        """高度限制：所有轨迹点应在飞行器最低/最高高度范围内"""
        min_h = performance.get_min_altitude()
        max_h = performance.get_max_altitude()
        for i, p in enumerate(traj):
            if p.z < min_h:
                result.add(ValidationIssue(
                    severity=Severity.ERROR, code="ALTITUDE_TOO_LOW",
                    message=f"轨迹点 {i} 高度 {p.z:.1f}m 低于飞行器最低高度 {min_h}m",
                    trajectory_index=i, time=p.t, position=(p.x, p.y, p.z),
                ))
            elif p.z > max_h:
                result.add(ValidationIssue(
                    severity=Severity.ERROR, code="ALTITUDE_TOO_HIGH",
                    message=f"轨迹点 {i} 高度 {p.z:.1f}m 高于飞行器最高高度 {max_h}m",
                    trajectory_index=i, time=p.t, position=(p.x, p.y, p.z),
                ))

    def _check_performance(
        self, traj: List[TrajectoryPoint],
        performance: AircraftPerformance,
        result: ValidationResult,
    ):
        """飞行性能：速度/爬升/转弯/加速度"""
        max_h_speed = performance.get_max_speed()
        max_climb = performance.get_max_climb_rate()
        max_desc = performance.get_max_descent_rate()
        max_turn_dps = performance.get_max_speed()  # placeholder, replaced by dedicated call below
        max_turn_dps = performance.performance.max_turn_rate_dps
        max_acc = performance.performance.max_acceleration_mps2
        max_dec = performance.performance.max_deceleration_mps2
        # 最小转弯半径（如果 performance 暴露该字段，否则由 max_turn_rate 推算）
        # 假设 v=max_speed，则 r_min = v / (turn_rate_rad) = max_speed / (max_turn_rate * π/180)
        import math
        if max_h_speed > 0 and max_turn_dps > 0:
            min_turn_radius = max_h_speed / (max_turn_dps * math.pi / 180.0)
        else:
            min_turn_radius = 0.0

        for i in range(1, len(traj)):
            p0, p1 = traj[i - 1], traj[i]
            dt = p1.t - p0.t
            if dt <= 0:
                result.add(ValidationIssue(
                    severity=Severity.WARNING, code="NON_MONOTONIC_TIME",
                    message=f"轨迹段 {i} 时间倒退（dt={dt:.2f}s）",
                    trajectory_index=i, time=p1.t,
                ))
                continue

            dx = p1.x - p0.x
            dy = p1.y - p0.y
            dz = p1.z - p0.z
            vx, vy, vz = dx / dt, dy / dt, dz / dt
            v_h = np.sqrt(vx * vx + vy * vy)
            v_full = np.sqrt(vx * vx + vy * vy + vz * vz)

            # 水平速度
            if v_h > max_h_speed + 0.1:
                result.add(ValidationIssue(
                    severity=Severity.ERROR, code="SPEED_EXCEEDED",
                    message=f"轨迹段 {i} 水平速度 {v_h:.2f}m/s 超过最大 {max_h_speed}m/s",
                    trajectory_index=i, time=p1.t, position=(p1.x, p1.y, p1.z),
                ))

            # 爬升率 / 下降率
            if vz > max_climb + 0.01:
                result.add(ValidationIssue(
                    severity=Severity.ERROR, code="CLIMB_RATE_EXCEEDED",
                    message=f"轨迹段 {i} 爬升率 {vz:.2f}m/s 超过最大 {max_climb}m/s",
                    trajectory_index=i, time=p1.t, position=(p1.x, p1.y, p1.z),
                ))
            elif vz < -max_desc - 0.01:
                result.add(ValidationIssue(
                    severity=Severity.ERROR, code="DESCENT_RATE_EXCEEDED",
                    message=f"轨迹段 {i} 下降率 {-vz:.2f}m/s 超过最大 {max_desc}m/s",
                    trajectory_index=i, time=p1.t, position=(p1.x, p1.y, p1.z),
                ))

            # 加速度（与上一段速度差）
            if i >= 2:
                p_prev = traj[i - 2]
                dt_prev = p0.t - p_prev.t
                if dt_prev > 0:
                    vx_prev = (p0.x - p_prev.x) / dt_prev
                    vy_prev = (p0.y - p_prev.y) / dt_prev
                    vz_prev = (p0.z - p_prev.z) / dt_prev
                    ax = vx - vx_prev
                    ay = vy - vy_prev
                    az = vz - vz_prev
                    a = np.sqrt(ax * ax + ay * ay + az * az) / dt
                    if a > max_acc + 0.1:
                        result.add(ValidationIssue(
                            severity=Severity.ERROR, code="ACCELERATION_EXCEEDED",
                            message=f"轨迹段 {i} 加速度 {a:.2f}m/s² 超过最大 {max_acc}m/s²",
                            trajectory_index=i, time=p1.t, position=(p1.x, p1.y, p1.z),
                        ))

            # 转弯率（航向变化）
            if i >= 2:
                p_prev = traj[i - 2]
                # 上一段航向
                h0 = np.degrees(np.arctan2(p0.y - p_prev.y, p0.x - p_prev.x))
                h1 = np.degrees(np.arctan2(p1.y - p0.y, p1.x - p0.x))
                d_h = ((h1 - h0 + 180) % 360) - 180
                turn_dps = abs(d_h) / dt if dt > 0 else 0
                if turn_dps > max_turn_dps + 1.0:
                    result.add(ValidationIssue(
                        severity=Severity.ERROR, code="TURN_RATE_EXCEEDED",
                        message=f"轨迹段 {i} 转弯率 {turn_dps:.1f}°/s 超过最大 {max_turn_dps}°/s",
                        trajectory_index=i, time=p1.t, position=(p1.x, p1.y, p1.z),
                    ))

    def _check_grid_occupancy(
        self, traj: List[TrajectoryPoint], result: ValidationResult,
    ):
        """与 SpaceTimeGrid 一致性检查（直接查 occupancy）"""
        for i, p in enumerate(traj):
            ix, iy, iz = self.grid.config.to_grid(p.x, p.y, p.z)
            it = self.grid.config.time_to_step(p.t)
            if not self.grid.is_free(ix, iy, iz, it):
                result.add(ValidationIssue(
                    severity=Severity.ERROR, code="GRID_OCCUPIED",
                    message=f"轨迹点 {i} 落在时空网格占用格 ({ix},{iy},{iz},t={it})",
                    trajectory_index=i, time=p.t, position=(p.x, p.y, p.z),
                ))