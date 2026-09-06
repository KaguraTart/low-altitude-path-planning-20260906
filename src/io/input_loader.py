"""
输入数据加载器

支持从 JSON 文件加载:
- 时空配置
- 静态障碍物数据
- 动态障碍物数据
- 飞行器性能配置
- 飞行任务定义
"""

from __future__ import annotations

import json
import os
from typing import Dict, List, Optional, Tuple

from ..models.space import SpaceTimeConfig
from ..models.obstacles import StaticObstacle, DynamicObstacle, ObstacleSet
from ..models.aircraft import AircraftPerformance, FlightTask, Waypoint


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


def load_aircraft_performances(filepath: str) -> Dict[str, AircraftPerformance]:
    """加载飞行器性能配置"""
    data = load_json(filepath)
    performances = {}
    for item in data.get("aircraft", data if isinstance(data, list) else []):
        aircraft_id = item.get("id", item.get("aircraft_id", "unknown"))
        performances[aircraft_id] = AircraftPerformance(
            aircraft_id=aircraft_id,
            model=item.get("model", "generic-quadcopter"),
            max_speed=float(item.get("max_speed", 15.0)),
            max_climb_rate=float(item.get("max_climb_rate", 5.0)),
            max_descent_rate=float(item.get("max_descent_rate", 3.0)),
            max_turn_rate=float(item.get("max_turn_rate", 45.0)),
            max_flight_time=float(item.get("max_flight_time", 1800.0)),
            max_range=float(item.get("max_range", 15000.0)),
            min_altitude=float(item.get("min_altitude", 10.0)),
            max_altitude=float(item.get("max_altitude", 300.0)),
        )
    return performances


def load_tasks(filepath: str) -> List[FlightTask]:
    """加载飞行任务"""
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
        ))
    return tasks
