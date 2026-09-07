"""
输入数据加载器

支持从 JSON 文件加载:
- 时空配置
- 静态障碍物数据
- 动态障碍物数据
- 飞行器性能配置（含扩展动力学字段：重量、尺寸、电源、续航等）
- 飞行任务定义（含任务需求约束）
"""

from __future__ import annotations

import json
import os
from typing import Dict, List, Optional, Tuple

from ..models.space import SpaceTimeConfig
from ..models.obstacles import StaticObstacle, DynamicObstacle, ObstacleSet
from ..models.aircraft import (
    AircraftPerformance,
    PhysicalSpec,
    PerformanceSpec,
    EnduranceSpec,
    OperationalSpec,
    SafetySpec,
    FlightTask,
    Waypoint,
    TaskRequirements,
)


def load_json(filepath: str) -> dict:
    """加载 JSON 文件"""
    with open(filepath, "r", encoding="utf-8") as f:
        return json.load(f)


def load_config(filepath: str) -> SpaceTimeConfig:
    """加载时空配置"""
    data = load_json(filepath)
    return SpaceTimeConfig(**data)


def load_static_obstacles(filepath: str) -> List[StaticObstacle]:
    """加载静态障碍物"""
    data = load_json(filepath)
    obstacles = []
    for item in data.get("obstacles", data if isinstance(data, list) else []):
        obstacles.append(StaticObstacle(
            obstacle_id=item.get("id", item.get("obstacle_id", "unknown")),
            name=item.get("name", "unnamed"),
            obstacle_type=item.get("type", item.get("obstacle_type", "other")),
            x=float(item["x"]),
            y=float(item["y"]),
            z=float(item.get("z", 0)),
            width=float(item.get("width", 20.0)),
            depth=float(item.get("depth", 20.0)),
            height=float(item.get("height", 50.0)),
            source=item.get("source", "file"),
        ))
    return obstacles


def load_dynamic_obstacles(filepath: str) -> List[DynamicObstacle]:
    """加载动态障碍物"""
    data = load_json(filepath)
    obstacles = []
    for item in data.get("obstacles", data if isinstance(data, list) else []):
        trajectory = []
        for point in item.get("trajectory", []):
            trajectory.append((
                float(point["x"]),
                float(point["y"]),
                float(point["z"]),
                float(point["t"]),
            ))
        obstacles.append(DynamicObstacle(
            obstacle_id=item.get("id", item.get("obstacle_id", "unknown")),
            name=item.get("name", "unnamed"),
            obstacle_type=item.get("type", item.get("obstacle_type", "other")),
            trajectory=trajectory,
            safety_radius=float(item.get("safety_radius", 10.0)),
            source=item.get("source", "file"),
        ))
    return obstacles


def load_obstacle_set(static_file: str, dynamic_file: Optional[str] = None) -> ObstacleSet:
    """加载完整障碍物集合"""
    obstacle_set = ObstacleSet()
    for obs in load_static_obstacles(static_file):
        obstacle_set.add_static(obs)
    if dynamic_file and os.path.exists(dynamic_file):
        for obs in load_dynamic_obstacles(dynamic_file):
            obstacle_set.add_dynamic(obs)
    return obstacle_set


def _parse_physical(d: dict) -> PhysicalSpec:
    return PhysicalSpec(
        weight_kg=float(d.get("weight_kg", 1.0)),
        max_takeoff_weight_kg=float(d.get("max_takeoff_weight_kg", 1.5)),
        payload_capacity_kg=float(d.get("payload_capacity_kg", 0.5)),
        wingspan_m=float(d.get("wingspan_m", 0.5)),
        length_m=float(d.get("length_m", 0.5)),
        width_m=float(d.get("width_m", 0.5)),
        height_m=float(d.get("height_m", 0.2)),
        rotor_radius_m=float(d.get("rotor_radius_m", 0.2)),
        diagonal_m=float(d.get("diagonal_m", 0.7)),
    )


def _parse_performance(d: dict) -> PerformanceSpec:
    return PerformanceSpec(
        max_speed_mps=float(d.get("max_speed_mps", d.get("max_speed", 15.0))),
        max_climb_rate_mps=float(d.get("max_climb_rate_mps", d.get("max_climb_rate", 5.0))),
        max_descent_rate_mps=float(d.get("max_descent_rate_mps", d.get("max_descent_rate", 3.0))),
        max_turn_rate_dps=float(d.get("max_turn_rate_dps", d.get("max_turn_rate", 45.0))),
        max_acceleration_mps2=float(d.get("max_acceleration_mps2", d.get("max_acceleration", 3.0))),
        max_deceleration_mps2=float(d.get("max_deceleration_mps2", 2.5)),
    )


def _parse_endurance(d: dict) -> EnduranceSpec:
    return EnduranceSpec(
        max_flight_time_s=float(d.get("max_flight_time_s", d.get("max_flight_time", 1800.0))),
        max_range_m=float(d.get("max_range_m", d.get("max_range", 15000.0))),
        battery_capacity_wh=float(d.get("battery_capacity_wh", 200.0)),
        cruise_power_w=float(d.get("cruise_power_w", 500.0)),
        hover_power_w=float(d.get("hover_power_w", 350.0)),
    )


def _parse_operational(d: dict) -> OperationalSpec:
    return OperationalSpec(
        min_altitude_m=float(d.get("min_altitude_m", d.get("min_altitude", 10.0))),
        max_altitude_m=float(d.get("max_altitude_m", d.get("max_altitude", 300.0))),
        min_operating_temp_c=float(d.get("min_operating_temp_c", -10.0)),
        max_operating_temp_c=float(d.get("max_operating_temp_c", 40.0)),
        max_wind_resistance_mps=float(d.get("max_wind_resistance_mps", 12.0)),
        ip_rating=str(d.get("ip_rating", "IP43")),
    )


def _parse_safety(d: dict) -> SafetySpec:
    return SafetySpec(
        rotor_radius_m=float(d.get("rotor_radius_m", 0.2)),
        safety_margin_m=float(d.get("safety_margin_m", 5.0)),
        geofence_compliant=bool(d.get("geofence_compliant", True)),
        return_to_home_altitude_m=float(d.get("return_to_home_altitude_m", 50.0)),
    )


def load_aircraft_performances(filepath: str) -> Dict[str, AircraftPerformance]:
    """加载飞行器性能配置（支持扩展动力学字段）"""
    data = load_json(filepath)
    performances = {}
    for item in data.get("aircraft", data if isinstance(data, list) else []):
        aircraft_id = item.get("id", item.get("aircraft_id", "unknown"))

        # 嵌套子对象
        physical = _parse_physical(item.get("physical", {}))
        performance = _parse_performance(item.get("performance", {}))
        endurance = _parse_endurance(item.get("endurance", {}))
        operational = _parse_operational(item.get("operational", {}))
        safety = _parse_safety(item.get("safety", {}))

        # 旧字段（向后兼容）
        max_speed = float(item.get("max_speed", performance.max_speed_mps))
        max_climb = float(item.get("max_climb_rate", performance.max_climb_rate_mps))
        max_desc = float(item.get("max_descent_rate", performance.max_descent_rate_mps))
        max_turn = float(item.get("max_turn_rate", performance.max_turn_rate_dps))
        max_acc = float(item.get("max_acceleration", performance.max_acceleration_mps2))
        max_time = float(item.get("max_flight_time", endurance.max_flight_time_s))
        max_range = float(item.get("max_range", endurance.max_range_m))
        min_alt = float(item.get("min_altitude", operational.min_altitude_m))
        max_alt = float(item.get("max_altitude", operational.max_altitude_m))
        rotor_r = float(item.get("rotor_radius", safety.rotor_radius_m))
        safety_m = float(item.get("safety_margin", safety.safety_margin_m))

        performances[aircraft_id] = AircraftPerformance(
            aircraft_id=aircraft_id,
            model=item.get("model", "generic-quadcopter"),
            category=item.get("category", "general"),
            manufacturer=item.get("manufacturer", "Unknown"),
            physical=physical,
            performance=performance,
            endurance=endurance,
            operational=operational,
            safety=safety,
            max_speed=max_speed,
            max_climb_rate=max_climb,
            max_descent_rate=max_desc,
            max_turn_rate=max_turn,
            max_acceleration=max_acc,
            max_flight_time=max_time,
            max_range=max_range,
            min_altitude=min_alt,
            max_altitude=max_alt,
            rotor_radius=rotor_r,
            safety_margin=safety_m,
        )
    return performances


def _parse_requirements(d: dict) -> TaskRequirements:
    delivery_window = d.get("delivery_window")
    if delivery_window and isinstance(delivery_window, list):
        delivery_window = tuple(delivery_window)
    return TaskRequirements(
        payload_kg=float(d.get("payload_kg", 0.0)),
        required_endurance_s=float(d.get("required_endurance_s", 0.0)),
        required_range_m=float(d.get("required_range_m", 0.0)),
        required_max_speed_mps=float(d.get("required_max_speed_mps", 0.0)),
        min_ceiling_m=float(d.get("min_ceiling_m", 0.0)),
        delivery_window=delivery_window,
        priority_level=int(d.get("priority_level", 1)),
        service_type=str(d.get("service_type", "delivery")),
    )


def load_tasks(filepath: str) -> List[FlightTask]:
    """加载飞行任务（含扩展需求约束）"""
    data = load_json(filepath)
    tasks = []
    for item in data.get("tasks", data if isinstance(data, list) else []):
        waypoints = []
        for wp_data in item.get("waypoints", []):
            waypoints.append(Waypoint(
                wp_id=wp_data.get("id", wp_data.get("wp_id", f"wp-{len(waypoints)}")),
                x=float(wp_data["x"]),
                y=float(wp_data["y"]),
                z=float(wp_data["z"]),
                earliest_arrival=wp_data.get("earliest_arrival"),
                latest_arrival=wp_data.get("latest_arrival"),
                hover_time=float(wp_data.get("hover_time", 0.0)),
                wp_type=wp_data.get("type", wp_data.get("wp_type", "via")),
                task_description=wp_data.get("task_description", ""),
            ))

        tasks.append(FlightTask(
            task_id=item.get("id", item.get("task_id", "unknown")),
            task_name=item.get("name", item.get("task_name", "unnamed")),
            aircraft_id=item.get("aircraft_id", "unknown"),
            waypoints=waypoints,
            priority=int(item.get("priority", 5)),
            task_type=item.get("type", item.get("task_type", "other")),
            departure_time=float(item.get("departure_time", 0.0)),
            objective_weights=item.get("objective_weights", {
                "distance": 0.3, "time": 0.3, "risk": 0.2, "energy": 0.2
            }),
            requirements=_parse_requirements(item.get("requirements", {})),
        ))
    return tasks